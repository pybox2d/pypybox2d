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

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import *
from .shapes import (Circle, Polygon, Edge, Loop, Shape)
from .settings import (DISTANCE_MAX_ITERS, EPSILON_SQR, EPSILON)

gjk_calls = 0
gjk_max_iters = 0
gjk_iters = 0

class SimplexCache(object):
    __slots__ = ['metric', 'count', 'indices']
    def __init__(self):
        self.count = 0
        self.metric = 0.0       # length or area
        self.indices = [(0, 0), (0, 0), (0, 0)]  # vertices on (shape a, shape b)

    def __repr__(self):
        return 'SimplexCache(metric=%g, count=%d, indices=%s)' \
                % (self.metric, self.count, self.indices)

class SimplexVertex(object):
    """
    wA     = support point in proxyA
    wB     = support point in proxyB
    w      = wB - wA
    a      = barycentric coordinate for closest point
    indexA = wA index
    indexB = wB index
    """
    __slots__ = ['wa', 'wb', 'w', 'a', 'index_a', 'index_b']
    def __init__(self, wa=(0,0), wb=(0,0), w=(0,0), a=0.0, index_a=0, index_b=0):
        self.wa = Vec2(*wa)
        self.wb = Vec2(*wb)
        self.w = Vec2(*w)
        self.a = a
        self.index_a = index_a
        self.index_b = index_b

    def __repr__(self):
        return 'SimplexVertex(wa=%s, wb=%s, w=%s, a=%s, index_a=%d, index_b=%d)' \
                % (self.wa, self.wb, self.w, self.a, self.index_a, self.index_b)

    def __copy__(self):
        return SimplexVertex(self.wa, self.wb, self.w, self.a, self.index_a, self.index_b)

class Simplex(object):
    __slots__ = ['_v1', '_v2', '_v3', '_count']
    def __init__(self):
        self._count = 0
        self._v1 = SimplexVertex()
        self._v2 = SimplexVertex()
        self._v3 = SimplexVertex()

    @property
    def vertices(self):
        return [self._v1, self._v2, self._v3]

    @property
    def used_vertices(self):
        return self.vertices[:self._count]

    def read_cache(self, cache, proxy_a, transform_a, proxy_b, transform_b):
        assert(cache.count <= 3)
        
        # Copy data from cache.
        self._count = cache.count
        vertices = self.vertices

        for i, v in enumerate(self.used_vertices):
            v.index_a, v.index_b = cache.indices[i]
            w_a_local = proxy_a._vertices[v.index_a]
            w_b_local = proxy_b._vertices[v.index_b]
            v.wa = transform_a * w_a_local
            v.wb = transform_b * w_b_local
            v.w = v.wb - v.wa
            v.a = 0.0

        # Compute the new simplex metric, if it is substantially different than
        # old metric then flush the simplex.
        if self._count > 1:
            metric1 = cache.metric
            metric2 = self.metric
            if metric2 < (0.5 * metric1) or (2.0 * metric1) < metric2 or metric2 < EPSILON:
                # Reset the simplex.
                self._count = 0
        
        # If the cache is empty or invalid ...
        if self._count == 0:
            v = vertices[0]
            v.index_a = 0
            v.index_b = 0
            w_a_local = proxy_a._vertices[0]
            w_b_local = proxy_b._vertices[0]
            v.wa = transform_a * w_a_local
            v.wb = transform_b * w_b_local
            v.w = v.wb - v.wa
            self._count = 1

    def write_cache(self, cache):
        cache.metric = self.metric
        cache.count = self._count
        vertices = self.vertices[:self._count]
        for i, vertex in enumerate(vertices):
            cache.indices[i] = (vertex.index_a, vertex.index_b)

    @property
    def closest_point(self):
        if self._count == 1:
            return copy(self._v1.w)
        elif self._count == 2:
            return self._v1.a * self._v1.w + self._v2.a * self._v2.w
        elif self._count == 3:
            return Vec2(0, 0)
        else:
            assert(False)
            return Vec2(0, 0)

    @property
    def search_direction(self):
        if self._count == 1:
            return -self._v1.w

        elif self._count == 2:
            e12 = self._v2.w - self._v1.w
            sgn = e12.cross(-self._v1.w)
            if sgn > 0.0:
                # Origin is left of e12
                return scalar_cross(1.0, e12)
            else:
                # Origin is right of e12
                return e12.cross(1.0)
        else:
            assert(False)
            return Vec2(0,0)

    @property
    def witness_points(self):
        if self._count == 1:
            return copy(self._v1.wa), copy(self._v1.wb)

        elif self._count == 2:
            return ((self._v1.a * self._v1.wa + self._v2.a * self._v2.wa),
                    (self._v1.a * self._v1.wb + self._v2.a * self._v2.wb))

        elif self._count == 3:
            vec=(self._v1.a * self._v1.wa + self._v2.a * self._v2.wa + self._v3.a * self._v3.wa)
            return (vec, copy(vec))

        else:
            assert(False)
            return Vec2(0,0), Vec2(0,0)

    @property
    def metric(self):
        if self._count == 1:
            return 0.0

        elif self._count == 2:
            return distance(self._v1.w, self._v2.w)

        elif self._count == 3:
            return (self._v2.w - self._v1.w).cross(self._v3.w - self._v1.w)

        else:
            assert(False)
            return 0.0

    def solve2(self):
        """
         Solve a line segment using barycentric coordinates.
        
         p = a1 * w1 + a2 * w2
         a1 + a2 = 1
        
         The vector from the origin to the closest point on the line is
         perpendicular to the line.
         e12 = w2 - w1
         dot(p, e) = 0
         a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        
         2-by-2 linear system
         [1      1     ][a1] = [1]
         [w1.e12 w2.e12][a2] = [0]
        
         Define
         d12_1 =  dot(w2, e12)
         d12_2 = -dot(w1, e12)
         d12 = d12_1 + d12_2
        
         Solution
         a1 = d12_1 / d12
         a2 = d12_2 / d12
        """
        w1 = self._v1.w
        w2 = self._v2.w
        e12 = w2 - w1

        # w1 region
        d12_2 = -w1.dot(e12)
        if d12_2 <= 0.0:
            # a2 <= 0, so we clamp it to 0
            self._v1.a = 1.0
            self._count = 1
            return

        # w2 region
        d12_1 = w2.dot(e12)
        if d12_1 <= 0.0:
            # a1 <= 0, so we clamp it to 0
            self._v2.a = 1.0
            self._count = 1
            self._v1 = copy(self._v2)
            return

        # Must be in e12 region.
        inv_d12 = 1.0 / (d12_1 + d12_2)
        self._v1.a = d12_1 * inv_d12
        self._v2.a = d12_2 * inv_d12
        self._count = 2

    def solve3(self):
        """
         Possible regions:
         - points[2]
         - edge points[0]-points[2]
         - edge points[1]-points[2]
         - inside the triangle
        """
        w1 = self._v1.w
        w2 = self._v2.w
        w3 = self._v3.w

        # Edge12
        # [1      1     ][a1] = [1]
        # [w1.e12 w2.e12][a2] = [0]
        # a3 = 0
        e12 = w2 - w1
        w1e12 = w1.dot(e12)
        w2e12 = w2.dot(e12)
        d12_1 = w2e12
        d12_2 = -w1e12

        # Edge13
        # [1      1     ][a1] = [1]
        # [w1.e13 w3.e13][a3] = [0]
        # a2 = 0
        e13 = w3 - w1
        w1e13 = w1.dot(e13)
        w3e13 = w3.dot(e13)
        d13_1 = w3e13
        d13_2 = -w1e13

        # Edge23
        # [1      1     ][a2] = [1]
        # [w2.e23 w3.e23][a3] = [0]
        # a1 = 0
        e23 = w3 - w2
        w2e23 = w2.dot(e23)
        w3e23 = w3.dot(e23)
        d23_1 = w3e23
        d23_2 = -w2e23
        
        # Triangle123
        n123 = e12.cross(e13)

        d123_1 = n123 * w2.cross(w3)
        d123_2 = n123 * w3.cross(w1)
        d123_3 = n123 * w1.cross(w2)

        # w1 region
        if d12_2 <= 0.0 and d13_2 <= 0.0:
            self._v1.a = 1.0
            self._count = 1
            return

        # e12
        if d12_1 > 0.0 and d12_2 > 0.0 and d123_3 <= 0.0:
            inv_d12 = 1.0 / (d12_1 + d12_2)
            self._v1.a = d12_1 * inv_d12
            self._v2.a = d12_2 * inv_d12
            self._count = 2
            return

        # e13
        if d13_1 > 0.0 and d13_2 > 0.0 and d123_2 <= 0.0:
            inv_d13 = 1.0 / (d13_1 + d13_2)
            self._v1.a = d13_1 * inv_d13
            self._v3.a = d13_2 * inv_d13
            self._count = 2
            self._v2 = copy(self._v3)
            return

        # w2 region
        if d12_1 <= 0.0 and d23_2 <= 0.0:
            self._v2.a = 1.0
            self._count = 1
            self._v1 = copy(self._v2)
            return

        # w3 region
        if d13_1 <= 0.0 and d23_1 <= 0.0:
            self._v3.a = 1.0
            self._count = 1
            self._v1 = copy(self._v3)
            return

        # e23
        if d23_1 > 0.0 and d23_2 > 0.0 and d123_1 <= 0.0:
            inv_d23 = 1.0 / (d23_1 + d23_2)
            self._v2.a = d23_1 * inv_d23
            self._v3.a = d23_2 * inv_d23
            self._count = 2
            self._v1 = copy(self._v3)
            return

        # Must be in triangle123
        inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3)
        self._v1.a = d123_1 * inv_d123
        self._v2.a = d123_2 * inv_d123
        self._v3.a = d123_3 * inv_d123
        self._count = 3

class DistanceProxy(object):
    """
    A distance proxy is used by the GJK algorithm.
    It encapsulates any shape.
    """
    __slots__ = ['_buffer', '_vertices', '_count', '_radius']
    def __init__(self, shape, index):
        """
        Initialize the proxy using the given shape. The shape
        must remain in scope while the proxy is in use.
        """
        if shape._type == Circle._type:
            circle = shape
            self._vertices = [copy(shape.position)]
            self._count = 1
            self._radius = circle.radius
            self._buffer = []

        elif shape._type == Polygon._type:
            polygon = shape
            self._vertices = [copy(v) for v in polygon._vertices]
            self._count = len(self._vertices)
            self._radius = polygon.radius
            self._buffer = []
        
        elif shape._type == Loop._type:
            loop = shape
            vertex_count = len(loop._vertices)
            assert(0 <= index < vertex_count)
            
            self._buffer = [loop._vertices[index], loop.vertices[(index + 1) % vertex_count]]
            self._vertices = self._buffer
            self._count = 2
            self._radius = loop.radius

        elif shape._type == Edge._type:
            edge = shape
            self._vertices = [edge._vertex1, edge._vertex2]
            self._count = 2
            self._radius = edge.radius
    
        else:
            assert(False)

    def get_support(self, d):
        """Get the supporting vertex index in the given direction."""
        dots = [vertex.dot(d) for vertex in self._vertices]
        return dots.index(max(dots))

    def get_support_vertex(self, d):
        """Get the supporting vertex in the given direction."""
        return copy(self._vertices[self.get_support(d)])
    
    def vertex_count(self):
        """Get the vertex count."""
        return self._count

    def vertex(self, i):
        return copy(self._vertices[i])

    @property
    def vertices(self):
        return self._vertices

def test_overlap(shape_a, index_a, shape_b, index_b, xf_a, xf_b):
    proxy_a = DistanceProxy(shape_a, index_a)
    proxy_b = DistanceProxy(shape_b, index_b)

    dist_info = shape_distance(proxy_a, proxy_b, xf_a, xf_b, True)
    point_a, point_b, distance, iterations = dist_info
    return distance < (10.0 * EPSILON)

def shape_distance(proxy_a, proxy_b, transform_a, transform_b, use_radii, cache=None):
    """
    Compute the closest points between two shapes. Supports any combination of:
    Circle, Polygon, Edge. The simplex cache is input/output, and optional.

    DistanceProxy is expected, but shapes can also be passed in directly. For
    repeated use, use DistanceProxy.
    @argument proxy_a DistanceProxy or shape (assuming child index 0)
    @argument proxy_b DistanceProxy or shape (assuming child index 0)
    @argument transform_a The transform to be applied to the first shape
    @argument transform_b The transform to be applied to the second shape
    @argument use_radii Consider radii in the calculation
    """
    if cache is None:
        cache = SimplexCache()
    
    if not isinstance(proxy_a, DistanceProxy):
        if not isinstance(proxy_a, Shape):
            raise ValueError('proxy_a: Must be either a DistanceProxy or a Shape')
        proxy_a = DistanceProxy(proxy_a, 0)
    if not isinstance(proxy_b, DistanceProxy):
        if not isinstance(proxy_b, Shape):
            raise ValueError('proxy_b: Must be either a DistanceProxy or a Shape')
        proxy_b = DistanceProxy(proxy_b, 0)

    global gjk_calls
    global gjk_max_iters
    global gjk_iters

    gjk_calls += 1

    # Initialize the simplex.
    simplex = Simplex()
    simplex.read_cache(cache, proxy_a, transform_a, proxy_b, transform_b)

    # Get simplex vertices
    vertices = simplex.vertices

    # These store the vertices of the last simplex so that we
    # can check for duplicates and prevent cycling.
    save_a=[0,0,0]
    save_b=[0,0,0]
    save_count = 0

    closest_point = simplex.closest_point
    distance_sqr1 = closest_point.length_squared
    distance_sqr2 = distance_sqr1

    # Main iteration loop.
    iter_ = 0
    while iter_ < DISTANCE_MAX_ITERS:
        # Copy simplex so we can identify duplicates.
        save_list = [(v.index_a, v.index_b) for v in simplex.used_vertices]

        if simplex._count == 1:
            pass
        elif simplex._count == 2:
            simplex.solve2()
        elif simplex._count == 3:
            simplex.solve3()
            # If we have 3 points, then the origin is in the corresponding triangle.
            break
        else:
            assert(False)

        # Compute closest point.
        p = simplex.closest_point
        distance_sqr2 = p.length_squared

        # Ensure progress
        if distance_sqr2 >= distance_sqr1:
            #break   # TODO: should be enabled?
            pass

        distance_sqr1 = distance_sqr2

        # Get search direction.
        d = simplex.search_direction

        # Ensure the search direction is numerically fit.
        if d.length_squared < EPSILON_SQR:
            # The origin is probably contained by a line segment
            # or triangle. Thus the shapes are overlapped.

            # We can't return zero here even though there may be overlap.
            # In case the simplex is a point, segment, or triangle it is difficult
            # to determine if the origin is contained in the CSO or very close to it.
            break

        # Compute a tentative new simplex vertex using support points.
        vertex = vertices[simplex._count]
        vertex.index_a = proxy_a.get_support(transform_a._rotation.mul_t(-d))
        vertex.wa = transform_a * (proxy_a._vertices[vertex.index_a])

        vertex.index_b = proxy_b.get_support(transform_b._rotation.mul_t(d))
        vertex.wb = transform_b * (proxy_b._vertices[vertex.index_b])
        vertex.w = vertex.wb - vertex.wa

        # Iteration count is equated to the number of support point calls.
        iter_ += 1
        gjk_iters += 1

        # Check for duplicate support points. This is the main termination criteria.
        if (vertex.index_a, vertex.index_b) in save_list:
            # If we found a duplicate support point we must exit to avoid cycling.
            break

        # New vertex is ok and needed.
        simplex._count += 1

    gjk_max_iters = max(gjk_max_iters, iter_)

    # Prepare output.
    point_a, point_b = simplex.witness_points
    distance_ = distance(point_a, point_b)

    # Cache the simplex.
    simplex.write_cache(cache)

    # Apply radii if requested.
    if use_radii:
        r_a = proxy_a._radius
        r_b = proxy_b._radius

        if distance_ > (r_a + r_b) and distance_ > EPSILON:
            # Shapes are still not overlapped.
            # Move the witness points to the outer surface.
            distance_ -= r_a + r_b
            normal = point_b - point_a
            normal.normalize()
            point_a += r_a * normal
            point_b -= r_b * normal
        else:
            # Shapes are overlapped when radii are considered.
            # Move the witness points to the middle.
            p = 0.5 * (point_a + point_b)
            point_a = p
            point_b = Vec2(*p)
            distance_ = 0.0
   
    return point_a, point_b, distance_, iter_ 
