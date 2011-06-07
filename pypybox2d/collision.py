#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
# Python port by Ken Lauer / http://pybox2d.googlecode.com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

from __future__ import absolute_import

__all__ = ('ABSOLUTE_TOL', 'RELATIVE_TOL',
           'ISOLATED', 'CONCAVE', 'FLAT', 'CONVEX',
           'collide_circles', 'collide_polygon_circle', 
           'collide_polygons', 'collide_edge_circle', 'collide_edge_polygon',
           'EPAxis', 'EPFatEdge', 'EPProxy', 'EPCollider'
           )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from .common import (Vec2, distance_squared)
from .collision_util import (max_separation, find_incident_edge, clip_segment_to_line)
from .contact_util import (ManifoldPoint, ClipVertex, Manifold)
from .settings import (MAX_FLOAT, EPSILON, MAX_MANIFOLD_POINTS, POLYGON_RADIUS, ANGULAR_SLOP)

# -- defines --
RELATIVE_TOL = 0.98
ABSOLUTE_TOL = 0.001

# Edge types
ISOLATED, CONCAVE, FLAT, CONVEX = range(4)

# -- circles --
def collide_circles(manifold, circle_a, xf_a, circle_b, xf_b):
    manifold.point_count = 0
    pa = xf_a * circle_a.position
    pb = xf_b * circle_b.position

    d = pb - pa
    dist_sqr = d.length_squared
    ra = circle_a.radius
    rb = circle_b.radius
    radius = ra + rb
    if dist_sqr > radius**2:
        return

    manifold.type = Manifold.CIRCLES
    manifold.local_point.set(*circle_a.position)
    manifold.local_normal.zero()
    manifold.point_count = 1

    point=manifold.points[0]
    point.local_point.set(*circle_b.position)
    point.set_id(0,0,0,0)

def collide_polygon_circle(manifold, polygon_a, xf_a, circle_b, xf_b):
    manifold.point_count = 0
    
    # Compute circle position in the frame of the polygon
    c = xf_b * circle_b.position
    c_local = xf_a.mul_t(c)

    # Find the min separating edge
    normal_index = 0
    separation = -MAX_FLOAT
    radius = polygon_a.radius + circle_b.radius

    vertices = polygon_a._vertices
    normals = polygon_a._normals

    vertex_count = len(vertices)
   
    for i in range(vertex_count):
        s = normals[i].dot(c_local - vertices[i])

        if s > radius:
            # Early out
            return
        if s > separation:
            separation = s
            normal_index = i

    # Vertices that subtend the incident face
    i1 = normal_index
    i2 = (i1 + 1) % vertex_count
    v1 = vertices[i1]
    v2 = vertices[i2]

    # If the center is inside the polygon...
    if separation < EPSILON:
        manifold.point_count = 1
        manifold.type = Manifold.FACE_A
        manifold.local_normal.set(*normals[normal_index])
        manifold.local_point = 0.5 * (v1 + v2)
        point=manifold.points[0]
        point.local_point.set(*circle_b.position)
        point.set_id(0, 0, 0, 0)
        return

    # Compute the barycentric coordinates
    u1 = (c_local - v1).dot(v2 - v1)
    u2 = (c_local - v2).dot(v1 - v2)

    if u1 < 0.0:
        if distance_squared(c_local, v1) > radius**2:
            return

        manifold.local_normal = c_local - v1
        manifold.local_normal.normalize()
        manifold.local_point.set(*v1)
    elif u2 < 0.0:
        if distance_squared(c_local, v2) > radius**2:
            return

        manifold.local_normal = c_local - v2
        manifold.local_normal.normalize()
        manifold.local_point.set(*v2)
    else:
        face_center = 0.5 * (v1 + v2)
        separation = (c_local - face_center).dot(normals[i1])
        if separation > radius:
            return

        manifold.local_normal.set(*normals[i1])
        manifold.local_point = face_center

    manifold.point_count = 1
    manifold.type = Manifold.FACE_A
    point=manifold.points[0]
    point.local_point.set(*circle_b.position)
    point.set_id(0, 0, 0, 0)

# -- polygons --
def collide_polygons(manifold, poly_a, xf_a, poly_b, xf_b):
    # Find edge normal of max separation on A - return if separating axis is found
    # Find edge normal of max separation on B - return if separation axis is found
    # Choose reference edge as min(minA, minB)
    # Find incident edge
    # Clip
    #                                                                              
    # The normal points from 1 to 2

    manifold.point_count = 0
    total_radius = poly_a.radius + poly_b.radius

    separation_a, edge_a = max_separation(poly_a, xf_a, poly_b, xf_b)
    if separation_a > total_radius:
        return

    separation_b, edge_b = max_separation(poly_b, xf_b, poly_a, xf_a)
    if separation_b > total_radius:
        return

    # poly1: reference polygon
    # poly2: incident polygon
    # edge1: reference edge

    if separation_b > (RELATIVE_TOL * separation_a + ABSOLUTE_TOL):
        poly1 = poly_b
        poly2 = poly_a
        xf1 = xf_b
        xf2 = xf_a
        edge1 = edge_b
        manifold.type = Manifold.FACE_B
        flip = True
    else:
        poly1 = poly_a
        poly2 = poly_b
        xf1 = xf_a
        xf2 = xf_b
        edge1 = edge_a
        manifold.type = Manifold.FACE_A
        flip = False

    incident_edge = find_incident_edge(poly1, xf1, edge1, poly2, xf2)

    vertices1 = poly1._vertices

    iv1 = edge1
    iv2 = (edge1 + 1) % len(poly1._vertices)

    v11 = vertices1[iv1]
    v12 = vertices1[iv2]

    localtangent = v12 - v11
    localtangent.normalize()
    
    local_normal = localtangent.cross(1.0)
    planepoint = 0.5 * (v11 + v12)

    tangent = xf1._rotation * localtangent
    normal = tangent.cross(1.0)
    
    v11 = xf1 * v11
    v12 = xf1 * v12

    # Face offset.
    front_offset = normal.dot(v11)

    # Side offsets, extended by polytope skin thickness.
    side_offset1 = -tangent.dot(v11) + total_radius
    side_offset2 = tangent.dot(v12) + total_radius

    # Clip incident edge against extruded edge1 side edges.
    # Clip to box side 1
    clip_points1 = clip_segment_to_line(incident_edge, -tangent, side_offset1, iv1)
    if len(clip_points1) < 2:
        return

    # Clip to negative box side 1
    clip_points2 = clip_segment_to_line(clip_points1, tangent, side_offset2, iv2)
    if len(clip_points2) < 2:
        return

    # Now clip_points2 contains the clipped points.
    manifold.local_normal = local_normal
    manifold.local_point = planepoint

    point_count = 0
    for i in range(MAX_MANIFOLD_POINTS):
        separation = normal.dot(clip_points2[i].v) - front_offset

        if separation <= total_radius:
            cp = manifold.points[point_count]
            cp.local_point = xf2.mul_t(clip_points2[i].v)
            cp.set_id(*clip_points2[i].id)
            if flip:
                # Swap features
                cp.set_id(cp.index_b, cp.index_a, cp.type_b, cp.type_a) 
            point_count += 1

    manifold.point_count = point_count


# -- edge --

def collide_edge_circle(manifold, edge_a, xf_a, circle_b, xf_b):
    """
    Compute contact points for edge versus circle.
    This accounts for edge connectivity.
    """
    manifold.point_count = 0

    # Compute circle in frame of edge
    Q = xf_a.mul_t(xf_b * circle_b.position)

    A = Vec2(*edge_a._vertex1)
    B = Vec2(*edge_a._vertex2)
    e = B - A

    # Barycentric coordinates
    u = e.dot(B - Q)
    v = e.dot(Q - A)

    radius = edge_a.radius + circle_b.radius

    index_b = 0
    type_b = ManifoldPoint.VERTEX

    # Region A
    if v <= 0.0:
        P = A
        d = Q - P
        dd = d.dot(d)
        if dd > radius**2:
            return

        # Is there an edge connected to A?
        if edge_a._vertex0 is not None:
            A1 = edge_a._vertex0
            B1 = A
            e1 = B1 - A1
            u1 = e1.dot(B1 - Q)

            # Is the circle in Region AB of the previous edge?
            if u1 > 0.0:
                return

        index_a = 0
        type_a = ManifoldPoint.VERTEX
        manifold.point_count = 1
        manifold.type = Manifold.CIRCLES
        manifold.local_normal.zero()
        manifold.local_point = P
        mp = manifold.points[0]
        mp.set_id(index_a, index_b, type_a, type_b) 
        mp.local_point = Vec2(*circle_b.position)
        return
    
    # Region B
    if u <= 0.0:
        P = B
        d = Q - P
        dd = d.dot(d)
        if dd > radius**2:
            return

        # Is there an edge connected to B?
        if edge_a._vertex3 is not None:
            B2 = edge_a._vertex3
            A2 = B
            e2 = B2 - A2
            v2 = e2.dot(Q - A2)

            # Is the circle in Region A_b of the next edge?
            if v2 > 0.0:
                return

        index_a = 1
        type_a =ManifoldPoint.VERTEX
        manifold.point_count = 1
        manifold.type = Manifold.CIRCLES
        manifold.local_normal.zero()
        manifold.local_point = P

        mp = manifold.points[0]
        mp.set_id(index_a, index_b, type_a, type_b) 
        mp.local_point = Vec2(*circle_b.position)
        return

    # Region A_b
    den = e.dot(e)
    assert(den > 0.0)
    P = (1.0 / den) * (u * A + v * B)
    d = Q - P
    dd = d.dot(d)
    if dd > radius**2:
        return

    n = Vec2(-e.y, e.x)
    if n.dot(Q - A) < 0.0:
        n = Vec2(-n.x, -n.y)
    n.normalize()

    index_a = 0
    type_a = ManifoldPoint.FACE
    manifold.point_count = 1
    manifold.type = Manifold.FACE_A
    manifold.local_normal = n
    manifold.local_point = A

    mp = manifold.points[0]
    mp.set_id(index_a, index_b, type_a, type_b) 
    mp.local_point = Vec2(*circle_b.position)

def collide_edge_polygon(manifold, edge_a, xf_a, polygon_b, xf_b):
    collider = EPCollider(edge_a, xf_a, polygon_b, xf_b)
    collider.collide(manifold)

# -- edge and polygon --
class EPAxis(object):
    __slots__ = ['type', 'index', 'separation']

    # Edge-polygon axis
    UNKNOWN, EDGE_A, EDGE_B = range(3)
    def __init__(self):
        self.type = EPAxis.UNKNOWN
        self.index = 0
        self.separation = 0.0

class EPFatEdge(object):
    """Edge shape plus more stuff."""
    __slots__ = ['v0', 'v1', 'v2', 'v3', 'normal']
    def __init__(self):
        pass

class EPProxy(object):
    """
    This lets us treate and edge shape and a polygon in the same
    way in the SAT collider.
    """
    __slots__ = ['vertices', 'normals', 'centroid']
    def __init__(self):
        pass

class EPCollider(object):
    """
    This class collides and edge and a polygon, taking into account edge adjacency.

    Collide an edge and polygon. This uses the SAT and clipping to produce up to 2 contact points.
    Edge adjacency is handle to produce locally valid contact points and normals. This is intended
    to allow the polygon to slide smoothly over an edge chain.

    Algorithm
    1. Classify front-side or back-side collision with edge.
    2. Compute separation
    3. Process adjacent edges
    4. Classify adjacent edge as convex, flat, null, or concave
    5. Skip null or concave edges. Concave edges get a separate manifold.
    6. If the edge is flat, compute contact points as normal. Discard boundary points.
    7. If the edge is convex, compute it's separation.
    8. Use the minimum separation of up to three edges. If the minimum separation
       is not the primary edge, return.
    9. If the minimum separation is the primary edge, compute the contact points and return.
    """
    __slots__ = ['_xf', '_edge_a', '_proxy_a', '_proxy_b', '_radius',
                '_limit11', '_limit12', '_limit21', '_limit22']

    def __init__(self, edge_a, xf_a, polygon_b, xf_b):
        # Transform
        self._xf = xf_a.mul_t(xf_b)

        # Edge geometry
        self._edge_a = EPFatEdge()
        self._edge_a.v0 = edge_a._vertex0
        self._edge_a.v1 = edge_a._vertex1
        self._edge_a.v2 = edge_a._vertex2
        self._edge_a.v3 = edge_a._vertex3
        e = self._edge_a.v2 - self._edge_a.v1

        # Normal points outwards in CCW order.
        self._edge_a.normal = Vec2(e.y, -e.x)
        self._edge_a.normal.normalize()

        # Proxy for edge
        self._proxy_a = EPProxy()
        self._proxy_a.vertices = [self._edge_a.v1, self._edge_a.v2]
        self._proxy_a.normals = [self._edge_a.normal, -self._edge_a.normal]
        self._proxy_a.centroid = 0.5 * (self._edge_a.v1 + self._edge_a.v2)

        # Proxy for polygon
        self._proxy_b = EPProxy()
        self._proxy_b.centroid = self._xf * polygon_b._centroid
        self._proxy_b.vertices = [self._xf * vertex for vertex in polygon_b._vertices]
        self._proxy_b.normals = [self._xf._rotation * normal for normal in polygon_b._normals]

        self._radius = 2.0 * POLYGON_RADIUS

        self._limit11 = Vec2()
        self._limit12 = Vec2()
        self._limit21 = Vec2()
        self._limit22 = Vec2()

    def collide(self, manifold):
        manifold.point_count = 0

        self.compute_adjacency()

        edge_axis = self.compute_edge_separation()

        # If no valid normal can be found than this edge should not collide.
        # This can happen on the middle edge of a 3-edge zig-zag chain.
        if edge_axis.type == EPAxis.UNKNOWN:
            return

        if edge_axis.separation > self._radius:
            return

        polygon_axis = self.compute_polygon_separation()
        if polygon_axis.type != EPAxis.UNKNOWN and polygon_axis.separation > self._radius:
            return

        # Use hysteresis for jitter reduction.
        if polygon_axis.type == EPAxis.UNKNOWN:
            primary_axis = edge_axis
        elif polygon_axis.separation > RELATIVE_TOL * edge_axis.separation + ABSOLUTE_TOL:
            primary_axis = polygon_axis
        else:
            primary_axis = edge_axis

        if primary_axis.type == EPAxis.EDGE_A:
            proxy1 = self._proxy_a
            proxy2 = self._proxy_b
            manifold.type = Manifold.FACE_A
        else:
            proxy1 = self._proxy_b
            proxy2 = self._proxy_a
            manifold.type = Manifold.FACE_B

        edge1 = primary_axis.index

        incident_edge = self.find_incident_edge(proxy1, primary_axis.index, proxy2)
        vertices1 = proxy1.vertices

        iv1 = edge1
        iv2 = (edge1 + 1) % len(proxy1.vertices)

        v11 = vertices1[iv1]
        v12 = vertices1[iv2]

        tangent = v12 - v11
        tangent.normalize()
        
        normal = tangent.cross(1.0)
        plane_point = 0.5 * (v11 + v12)

        # Face offset.
        front_offset = normal.dot(v11)

        # Side offsets, extended by polytope skin thickness.
        side_offset1 = -tangent.dot(v11) + self._radius
        side_offset2 =  tangent.dot(v12) + self._radius

        # Clip incident edge against extruded edge1 side edges.
        # Clip to box side 1
        clip_points1 = clip_segment_to_line(incident_edge, -tangent, side_offset1, iv1)
        if len(clip_points1) < MAX_MANIFOLD_POINTS:
            return

        # Clip to negative box side 1
        clip_points2 = clip_segment_to_line(clip_points1, tangent, side_offset2, iv2)
        if len(clip_points2) < MAX_MANIFOLD_POINTS:
            return

        # Now clip_points2 contains the clipped points.
        if primary_axis.type == EPAxis.EDGE_A:
            manifold.local_normal = normal
            manifold.local_point = plane_point
        else:
            manifold.local_normal = self._xf._rotation.mul_t(normal)
            manifold.local_point = self._xf.mul_t(plane_point)
    
        point_count = 0
        for clip_point in clip_points2:
            separation = normal.dot(clip_point.v) - front_offset

            if separation <= self._radius:
                cp = manifold.points[point_count]

                if primary_axis.type == EPAxis.EDGE_A:
                    cp.local_point = self._xf.mul_t(clip_point.v)
                    cp.id = tuple(clip_point.id)
                else:
                    cp.local_point = clip_point.v
                    cp.set_id(clip_point.type_b, clip_point.type_a, clip_point.index_b, clip_point.index_a)

                point_count += 1
        manifold.point_count = point_count

    # Compute allowable normal ranges based on adjacency.
    # A normal n is allowable iff:
    # cross(n, n1) >= 0.0 and cross(n2, n) >= 0.0
    # n points from A to B (edge to polygon)
    def compute_adjacency(self):
        v0 = self._edge_a.v0
        v1 = self._edge_a.v1
        v2 = self._edge_a.v2
        v3 = self._edge_a.v3

        # Determine allowable the normal regions based on adjacency.
        # Note: it may be possible that no normal is admissable.
        center_b = self._proxy_b.centroid
        if self._edge_a.v0 is not None:
            e0 = v1 - v0
            e1 = v2 - v1
            n0 = Vec2(e0.y, -e0.x)
            n1 = Vec2(e1.y, -e1.x)
            n0.normalize()
            n1.normalize()

            convex = n0.cross(n1) >= 0.0
            front0 = n0.dot(center_b - v0) >= 0.0
            front1 = n1.dot(center_b - v1) >= 0.0

            if convex:
                if front0 or front1:
                    self._limit11 = n1
                    self._limit12 = n0
                else:
                    self._limit11 = -n1
                    self._limit12 = -n0
            else:
                if front0 and front1:
                    self._limit11 = n0
                    self._limit12 = n1
                else:
                    self._limit11 = -n0
                    self._limit12 = -n1
        else:
            self._limit11.zero()
            self._limit12.zero()

        if self._edge_a.v3 is not None:
            e1 = v2 - v1
            e2 = v3 - v2
            n1 = Vec2(e1.y, -e1.x)
            n2 = Vec2(e2.y, -e2.x)
            n1.normalize()
            n2.normalize()

            convex = n1.cross(n2) >= 0.0
            front1 = n1.dot(center_b - v1) >= 0.0
            front2 = n2.dot(center_b - v2) >= 0.0

            if convex:
                if front1 or front2:
                    self._limit21 = n2
                    self._limit22 = n1
                else:
                    self._limit21 = -n2
                    self._limit22 = -n1
            else:
                if front1 and front2:
                    self._limit21 = n1
                    self._limit22 = n2
                else:
                    self._limit21 = -n1
                    self._limit22 = -n2
        else:
            self._limit21.zero()
            self._limit22.zero()

    def compute_edge_separation(self):
        # Edge_a separation
        best_axis = EPAxis()
        best_axis.type = EPAxis.UNKNOWN
        best_axis.index = -1
        best_axis.separation = -MAX_FLOAT
        normals = [self._edge_a.normal, -self._edge_a.normal]
        
        for i, n in enumerate(normals):
            # Adjacency
            valid1 = (n.cross(self._limit11) >= -ANGULAR_SLOP) and (self._limit12.cross(n) >= -ANGULAR_SLOP)
            valid2 = (n.cross(self._limit21) >= -ANGULAR_SLOP) and (self._limit22.cross(n) >= -ANGULAR_SLOP)

            if not valid1 or not valid2:
                continue
            
            axis = EPAxis()
            axis.type = EPAxis.EDGE_A
            axis.index = i

            dots = [n.dot(vertex - self._edge_a.v1) for vertex in self._proxy_b.vertices]
            axis.separation = min(dots)

            if axis.separation > self._radius:
                return axis

            if axis.separation > best_axis.separation:
                best_axis = axis

        return best_axis

    def compute_polygon_separation(self):
        axis = EPAxis()
        axis.type = EPAxis.UNKNOWN
        axis.index = -1
        axis.separation = -MAX_FLOAT
        for i in range(len(self._proxy_b.vertices)):
            n = -self._proxy_b.normals[i]

            # Adjacency
            valid1 = (n.cross(self._limit11) >= -ANGULAR_SLOP) and (self._limit12.cross(n) >= -ANGULAR_SLOP)
            valid2 = (n.cross(self._limit21) >= -ANGULAR_SLOP) and (self._limit22.cross(n) >= -ANGULAR_SLOP)

            if not valid1 or not valid2:
                continue

            s1 = n.dot(self._proxy_b.vertices[i] - self._edge_a.v1)
            s2 = n.dot(self._proxy_b.vertices[i] - self._edge_a.v2)
            s = min(s1, s2)

            if s > self._radius:
                axis.type = EPAxis.EDGE_B
                axis.index = i
                axis.separation = s

            if s > axis.separation:
                axis.type = EPAxis.EDGE_B
                axis.index = i
                axis.separation = s

        return axis

    def find_incident_edge(self, proxy1, edge1, proxy2):
        normals1 = proxy1.normals

        vertices2 = proxy2.vertices
        normals2 = proxy2.normals

        assert(0 <= edge1 < len(normals1))

        # Get the normal of the reference edge in proxy2's frame.
        normal1 = normals1[edge1]

        # Find the incident edge on proxy2.
        dots = [normal1.dot(v) for v in normals2]
        min_dot = min(dots)
        index = dots.index(min_dot)

        # Build the clip vertices for the incident edge.
        i1 = index
        i2 = (i1 + 1) % len(vertices2)

        c0=ClipVertex(vertices2[i1], edge1, i1, ManifoldPoint.FACE, ManifoldPoint.VERTEX)
        c1=ClipVertex(vertices2[i2], edge1, i2, ManifoldPoint.FACE, ManifoldPoint.VERTEX)
        return [c0, c1]

