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

import numbers
from copy import copy
from .common import *
from .settings import (EPSILON, EPSILON_SQR, POLYGON_RADIUS)

class MassData(object):
    """
    This holds the mass data computed for a shape.

    mass: The mass of the shape, usually in kilograms.
    center: The position of the shape's centroid relative to the shape's origin.
    I: The rotational inertia of the shape about the local origin.
    """
    def __init__(self, mass=0.0, center=(0,0), I=0.0):
        self.mass=mass
        self.center=Vec2(*center)
        self.I=I

    def __repr__(self):
        return 'MassData(mass=%g, center=%s, I=%g)' % (self.mass, self.center, self.I)

class Shape(object):
    """
    A shape is used for collision detection. You can create a shape however you like.
    Shapes used for simulation in World are created automatically when a Fixture
    is created. Shapes may encapsulate a one or more child shapes.

    Note: This is an abstract base class.
    """
    radius=0.0
    def __init__(self):
        raise NotImplementedError('Abstract class')
    def __copy__(self):
        return NotImplementedError # Abstract
    def __repr__(self):
        return NotImplementedError # Abstract
    def test_point(self, xf, p):
        """
        Test a point for containment in this shape. This only works for convex shapes.
        @param xf the shape world transform.
        @param p a point in world coordinates.
        """
        return NotImplementedError # Abstract

    def ray_cast(self, p1, p2, max_fraction, transform, child_index):
        """
        Cast a ray against a child shape.
        Returns: (hit, normal, fraction)
        @param p1 point 1
        @param p2 point 2
        @param max_fraction maximum fraction
        @param transform the transform to be applied to the shape.
        @param childIndex the child shape index
        """
        return NotImplementedError # Abstract

    def compute_aabb(self, xf, child_index, use_instance=None):
        """
        Given a transform, compute the associated axis aligned bounding box for a child shape.
        @param xf the world transform of the shape.
        @param childIndex the child shape
        @param use_instance if not None, uses this instance for the return value
        """
        return NotImplementedError # Abstract

    def compute_mass(self, density, use_instance=None):
        """
        Compute the mass properties of this shape using its dimensions and density.
        The inertia tensor is computed about the local origin.
        @param massData returns the mass data for this shape.
        @param density the density in kilograms per meter squared.
        @param use_instance if not None, uses this instance for the return value
        """
        return NotImplementedError # Abstract

    @property
    def vertex_count(self):
        return NotImplementedError # Abstract
    @property
    def child_count(self):
        return NotImplementedError # Abstract

class Circle(Shape):
    def __init__(self, radius=0.2, position=(0,0)):
        self.position=Vec2(*position)
        self.radius=radius
        # TODO radius should be _radius, position also:
        #  and add properties

    def __copy__(self):
        return Circle(self.radius, self.position)

    def __repr__(self):
        return 'Circle(radius=%g, position=%s)' % (self.radius, self.position)

    def test_point(self, transform, p):
        """See Shape.test_point"""
        center=transform.position + transform._rotation*self.position
        d=p-center
        return (d*d) <= self.radius**2

    def ray_cast(self, p1, p2, max_fraction, transform, child_index):
        """See Shape.ray_cast"""
        # Collision Detection in Interactive 3D Environments by Gino van den Bergen
        # From Section 3.1.2
        # x = s + a * r
        # norm(x) = radius
        position = transform.position + transform._rotation*self.position
        s = p1 - position
        b = s * s - self.radius**2

        # Solve quadratic equation.
        r = Vec2(*p2) - p1
        c =  s * r
        rr = r * r
        sigma = c * c - rr * b

        # Check for negative discriminant and short segment.
        if sigma < 0.0 or rr < EPSILON:
            return False, None, 0.0

        # Find the point of intersection of the line with the circle.
        a = -(c + math.sqrt(sigma))

        # Is the intersection point on the segment?
        if 0.0 <= a and a <= max_fraction * rr:
            a /= rr
            fraction = a
            normal = s + a * r
            normal.normalize()
            return True, normal, fraction
        return False, None, 0.0

    def compute_aabb(self, xf, child_index, use_instance=None):
        """See Shape.compute_aabb"""
        if use_instance:
            aabb=use_instance
        else:
            aabb=AABB()
        p = xf.position + xf._rotation*self.position
        aabb.lower_bound.set(p.x - self.radius, p.y - self.radius)
        aabb.upper_bound.set(p.x + self.radius, p.y + self.radius)
        return aabb

    def compute_mass(self, density, use_instance=None):
        """See Shape.compute_mass"""
        if use_instance:
            md=use_instance
        else:
            md=MassData()

        md.mass=density * PI * self.radius**2
        md.center=copy(self.position)
        md.I = md.mass * (0.5 * self.radius**2 + self.position*self.position)
        return md

    @property
    def vertex_count(self):
        return 1
    def get_support(self, vec):
        return 0
    def get_support_vertex(self, vec):
        return copy(self.position)
    def get_vertex(self, index):
        return copy(self.position)
    @property
    def child_count(self):
        return 1
    @property
    def vertices(self):
        return [copy(self.position)]

class Polygon(Shape):
    def __init__(self, vertices=None, box=None):
        self.radius=POLYGON_RADIUS
        self._vertices=[]
        self._normals=[]
        self._centroid=Vec2()
        if box:
            self.set_as_box(*box)
        elif vertices is not None:
            self.vertices=[Vec2(*v) for v in vertices]

    def __copy__(self):
        return Polygon(self._vertices)

    def __repr__(self):
        return 'Polygon(%s)' % self._vertices

    def test_point(self, xf, p):
        """See Shape.test_point"""
        p_local=xf._rotation.mul_t(p - xf.position)
        for normal, vertex in zip(self._normals, self._vertices):
            dot=normal.dot(p_local - vertex)
            if dot > 0.0:
                return False
        return True

    def ray_cast(self, p1, p2, max_fraction, xf, child_index):
        """See Shape.ray_cast"""

        # Put the ray into the polygon's frame of reference.
        p1 = xf._rotation.mul_t(p1 - xf.position)
        p2 = xf._rotation.mul_t(p2 - xf.position)
        d = p2 - p1
        
        lower, upper = 0.0, max_fraction
        tagged_normal=None
        for normal, vertex in zip(self._normals, self._vertices):
            # p = p1 + a * d
            # dot(normal, p - v) = 0
            # dot(normal, p1 - v) + a * dot(normal, d) = 0
            numerator=normal * (vertex - p1)
            denominator=normal * d

            if denominator==0.0:
                if numerator < 0.0:
                    return False, None, 0.0

            else:
                # Note: we want this predicate without division:
                # lower < numerator / denominator, where denominator < 0
                # Since denominator < 0, we have to flip the inequality:
                # lower < numerator / denominator <==> denominator * lower > numerator.
                if denominator < 0.0 and numerator < lower * denominator:
                    # Increase lower.
                    # The segment enters this half-space.
                    lower = numerator / denominator
                    tagged_normal=normal
                elif denominator > 0.0 and numerator < upper * denominator:
                    # Decrease upper.
                    # The segment exits this half-space.
                    upper = numerator / denominator

            # The use of epsilon here causes the assert on lower to trip
            # in some cases. Apparently the use of epsilon was to make edge
            # shapes work, but now those are handled separately.
            if upper < lower:
                return False, None, 0.0

        assert(0.0 <= lower <= max_fraction)

        if tagged_normal is not None:
            normal = xf._rotation * tagged_normal
            fraction = lower
            return True, normal, fraction

        return False, None, 0.0

    def compute_aabb(self, xf, child_index, use_instance=None):
        """See Shape.compute_aabb"""
        if use_instance:
            aabb=use_instance
        else:
            aabb=AABB()

        lower = xf * self.vertices[0]
        upper = lower

        for vertex in self._vertices:
            v = xf * vertex
            lower = min_vector(lower, v)
            upper = max_vector(upper, v)

        r=(self.radius, self.radius)
        aabb.lower_bound = lower - r
        aabb.upper_bound = upper + r
        return aabb

    def compute_mass(self, density, use_instance=None):
        """See Shape.compute_mass"""
        if use_instance:
            md=use_instance
        else:
            md=MassData()

        # Polygon mass, centroid, and inertia.
        # Let rho be the polygon density in mass per unit area.
        # Then:
        # mass = rho * int(dA)
        # centroid.x = (1/mass) * rho * int(x * dA)
        # centroid.y = (1/mass) * rho * int(y * dA)
        # I = rho * int((x*x + y*y) * dA)
        #
        # We can compute these integrals by summing all the integrals
        # for each triangle of the polygon. To evaluate the integral
        # for a single triangle, we make a change of variables to
        # the (u,v) coordinates of the triangle:
        # x = x0 + e1x * u + e2x * v
        # y = y0 + e1y * u + e2y * v
        # where 0 <= u && 0 <= v && u + v <= 1.
        #
        # We integrate u from [0,1-v] and then v from [0,1].
        # We also need to use the Jacobian of the transformation:
        # D = cross(e1, e2)
        #
        # Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        #
        # The rest of the derivation is handled by computer algebra.
        if len(self.vertices) < 3:
            raise ValueError('Polygons are >= 3 vertices')

        center=Vec2()
        area = 0.0
        I = 0.0

        # s is the reference point for forming triangles.
        # It's location doesn't change the result (except for rounding error).
        s=Vec2()

        # This code would put the reference point inside the polygon.
        for vertex in self.vertices:
            s += vertex

        s *= 1.0 / len(self.vertices)

        inv3 = 1.0 / 3.0
        
        count=len(self.vertices)

        for i, vertex in enumerate(self.vertices):
            # Triangle vertices.
            next_vertex=self.vertices[(i+1) % count]
            e1 = vertex - s
            e2 = next_vertex - s

            D = e1.cross(e2)

            triangle_area = 0.5 * D
            area += triangle_area

            # Area weighted centroid
            center += triangle_area * inv3 * (e1 + e2)

            ex1 = e1.x
            ey1 = e1.y

            ex2 = e2.x
            ey2 = e2.y

            intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2
            inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2

            I += (0.25 * inv3 * D) * (intx2 + inty2)

        # Total mass
        md.mass = density * area

        # Center of mass
        assert(area > EPSILON)

        center *= 1.0 / area
        md.center = center + s

        # Inertia tensor relative to the local origin.
        md.I = density * I + md.mass * (s*s)
        return md

    @property
    def vertex_count(self):
        return 1
    def get_support(self, vec):
        return 0
    def get_support_vertex(self, vec):
        return copy(self.position)
    def get_vertex(self, index):
        return copy(self.position)
    @property
    def child_count(self):
        return 1
    @property
    def vertices(self):
        return self._vertices
    @vertices.setter
    def vertices(self, vertices):
        if len(vertices) < 3:
            raise ValueError('Polygons are >= 3 vertices')

        # Compute normals. Ensure the edges have non-zero length.
        count=len(vertices)
        normals=[]
        for i, vertex in enumerate(vertices):
            edge = vertices[(i + 1) % count] - vertices[i]
            if edge.length_squared <= EPSILON_SQR:
                raise ValueError('Edges are too short')
            normal=edge.cross(1.0)
            normal.normalize()
            normals.append(normal)

        self._normals=normals

        # Copy the vertices
        self._vertices=[Vec2(*v) for v in vertices]

        # And finally, compute the polygon centroid
        self.centroid=self.compute_centroid()

    def set_as_box(self, hx, hy, center=(0, 0), angle=0.0):
        self._vertices=[
            Vec2(-hx, -hy),
            Vec2( hx, -hy),
            Vec2( hx,  hy),
            Vec2(-hx,  hy),
        ]
        self._normals=[
            Vec2( 0, -1),
            Vec2( 1,  0),
            Vec2( 0,  1),
            Vec2(-1,  0),
        ]
        self.centroid=Vec2(*center)
        if center is not None:
            xf=Transform(center, angle=angle)
            self._vertices=[xf*v for v in self._vertices]
            self._normals=[xf*v for v in self._normals]

    def compute_centroid(self, vertices=None):
        if vertices is None:
            vertices=self._vertices
        if len(vertices) < 3:
            raise ValueError('Need >= 3 vertices to compute the centroid')

        # p_ref is the reference point for forming triangles.
        # Its location doesn't change the result (except for rounding error).
        p_ref=Vec2()
        c=Vec2()
        area=0.0
        inv3=1.0 / 3.0
        count=len(vertices)

        for i, v in enumerate(vertices):
            p1 = p_ref
            p2 = v
            p3 = vertices[(i + 1) % count]

            e1 = p2 - p1
            e2 = p3 - p1
            D = e1.cross(e2)

            triangle_area = 0.5 * D
            area += triangle_area

            c += triangle_area * inv3 * (p1 + p2 + p3)

        # Centroid
        if area <= EPSILON:
            raise ValueError('Area of the polygon is too small to get a centroid')
        
        c *= 1.0 / area
        return c

class Edge(Shape):
    """
    A line segment (edge) shape. These can be connected in chains or loops
    to other edge shapes. The connectivity information is used to ensure
    correct contact normals.
    """
    def __init__(self, v1=None, v2=None, v0=None, v3=None):
        """
        vertex1 and vertex2: These are the edge vertices

        vertex0 and vertex3 are optional adjacent vertices. These are used for smooth 
        collision.
        """
        self.radius = POLYGON_RADIUS
        self._vertex1 = Vec2(*v1)
        self._vertex2 = Vec2(*v2)
        if v0:
            self._vertex0 = Vec2(*v0)
        else:
            self._vertex0 = None

        if v3:
            self._vertex3 = Vec2(*v3)
        else:
            self._vertex3 = None

    def __copy__(self):
        return Edge(self._vertex1, self._vertex2, self._vertex0, self._vertex3)

    def __repr__(self):
        return 'Edge(vertex1=%s, vertex2=%s, [vertex0=%s, vertex3=%s])' % \
                (self._vertex1, self._vertex2, self._vertex0, self._vertex3)

    def test_point(self, xf, p):
        """This always return false."""
        return False

    def ray_cast(self, p1, p2, max_fraction, xf, child_index):
        """See Shape.ray_cast"""
        # p = p1 + t * d
        # v = v1 + s * e
        # p1 + t * d = v1 + s * e
        # s * e - t * d = p1 - v1
        # Put the ray into the edge's frame of reference.
        p1 = xf._rotation.mul_t(p1 - xf.position)
        p2 = xf._rotation.mul_t(p2 - xf.position)
        d = p2 - p1

        v1 = self._vertex1
        v2 = self._vertex2
        e = v2 - v1
        normal = Vec2(e.y, -e.x)
        normal.normalize()

        # q = p1 + t * d
        # dot(normal, q - v1) = 0
        # dot(normal, p1 - v1) + t * dot(normal, d) = 0
        numerator = normal.dot(v1 - p1)
        denominator = normal.dot(d)

        if denominator == 0.0:
            return False, None, 0.0

        t = numerator / denominator
        if t < 0.0 or 1.0 < t:
            return False, None, 0.0

        q = p1 + t * d

        # q = v1 + s * r
        # s = dot(q - v1, r) / dot(r, r)
        r = v2 - v1
        rr = r.dot(r)
        if rr == 0.0:
            return False, None, 0.0

        s = (q - v1).dot(r) / rr
        if s < 0.0 or 1.0 < s:
            return False, None, 0.0

        fraction = t
        if numerator > 0.0:
            normal = -normal
        return True, normal, fraction

    def compute_aabb(self, xf, child_index, use_instance=None):
        """See Shape.compute_aabb"""
        v1 = xf * self._vertex1
        v2 = xf * self._vertex2

        lower = min_vector(v1, v2)
        upper = max_vector(v1, v2)

        r = (self.radius, self.radius)
        return AABB(lower - r, upper + r)

    def compute_mass(self, density, use_instance=None):
        return MassData(mass=0.0, 
                        center=0.5 * (self._vertex1 + self._vertex2),
                        I = 0.0)

    @property
    def child_count(self):
        return 1
    @property
    def vertices(self):
        return [Vec2(*v) for v in (self._vertex0, self._vertex1, self._vertex2, self._vertex3)]

    @property
    def vertex0(self):
        if self._vertex0 is not None:
            return Vec2(*self._vertex0)
        else:
            return None
    @property
    def vertex1(self):
        return Vec2(*self._vertex1)
    @property
    def vertex2(self):
        return Vec2(*self._vertex2)
    @property
    def vertex3(self):
        if self._vertex3 is not None:
            return Vec2(*self._vertex3)
        else:
            return None

class Loop(Shape):
    """
    A loop shape is a free form sequence of line segments that form a circular list.
    The loop may cross upon itself, but this is not recommended for smooth collision.
    The loop has double sided collision, so you can use inside and outside collision.
    Therefore, you may use any winding order.
    """
    def __init__(self, vertices, *args):
        """
        Supports either
         Loop((v1x, v1y), (v2x, v2y), ...)
         or
         Loop([(v1x, v1y), (v2x, v2y), ...])
        """
        if len(vertices) == 2 and isinstance(vertices[0], numbers.Number):
            vertices = [vertices]
            vertices.extend(args)

        if len(vertices) < 2:
            raise ValueError('Must be >= 2 vertices')
        self.radius = POLYGON_RADIUS
        self._vertices = [Vec2(*v) for v in vertices]

    def __copy__(self):
        return Loop(self._vertices)

    def __repr__(self):
        return 'Loop(vertices=%s)' % self._vertices

    def test_point(self, xf, p):
        """This always return false."""
        return False

    def ray_cast(self, p1, p2, max_fraction, xf, child_index):
        """See Shape.ray_cast"""

        i1 = child_index
        i2 = (child_index + 1) % len(self._vertices)

        edge = Edge(self._vertices[i1], self._vertices[i2])
        return edge.ray_cast(p1, p2, max_fraction, xf, 0)

    def compute_aabb(self, xf, child_index, use_instance=None):
        """See Shape.compute_aabb"""
        i1 = child_index
        i2 = (child_index + 1) % len(self._vertices)

        v1 = xf * self._vertices[i1]
        v2 = xf * self._vertices[i2]

        return AABB(min_vector(v1, v2), max_vector(v1, v2))

    def compute_mass(self, density, use_instance=None):
        """Chains have zero mass."""
        return MassData()

    def child_edge(self, index):
        """Get a child edge"""
        count = len(self._vertices)
        assert(0 <= index < count)

        i0 = (index - 1) % count
        i1 = index
        i2 = (index + 1) % count
        i3 = index + 2
        while i3 >= count:
            i3 -= count

        return Edge(self._vertices[i1], self._vertices[i2], self._vertices[i0], self._vertices[i3])

    def get_vertex(self, index):
        return copy(self._vertices[index])
    @property
    def child_count(self):
        return len(self._vertices)
    @property
    def vertices(self):
        return [Vec2(*v) for v in self._vertices]

Shapes = [Circle, Polygon, Loop, Edge]
SHAPE_COUNT = len(Shapes)

for i, shape_class in enumerate(Shapes):
    shape_class._type = i
del i
del shape_class

if __name__=='__main__':
    c=Circle(radius=5)
    assert(c.test_point(Transform(), c.position))
    assert(not c.test_point(Transform(), c.position+(100,100)))
    assert(c.ray_cast((-10, 0), (0, 0), 1.0, Transform(), 0))

    p1=Polygon(vertices=[(-1,0), (0,-1), (1, 0), (0, 1)])
    print(p1.vertices)
    p2=Polygon(box=[1,1])
    print(p2.vertices)
    p3=Polygon(box=[1,1,(1,1),3])
    print(p3.vertices)
    edge=Edge((0,0), (10, 10))
    print(edge)
    loop=Loop((0,0), (10, 10), (20, 20))
    print(loop)
    for shape in [c, p1, p2, p3, edge, loop]:
        if hasattr(shape, 'compute_centroid'):
            shape.compute_centroid()
        md=shape.compute_mass(1.0)
        aabb=shape.compute_aabb(Transform(), 0)
        print(md, aabb)
        print(shape.ray_cast((-10, 0), (10, 0), 1.0, Transform(), 0))
        print(shape.test_point(Transform(), (0, 0)))
