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

__all__ = ('mix_friction', 'mix_restitution', 
           'ManifoldPoint', 'ClipVertex', 'Manifold', 'WorldManifold', 
           'ContactRegister', 'ContactConstraintPoint', 'ContactConstraint')

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

import math
from copy import copy
from .common import (Mat22, Vec2, distance_squared)
from .settings import (MAX_MANIFOLD_POINTS, EPSILON_SQR)

def mix_friction(friction1, friction2):
    """
    Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
    For example, anything slides on ice.
    """
    return math.sqrt(friction1 * friction2)

def mix_restitution(restitution1, restitution2):
    """
    Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
    For example, a superball bounces on anything.
    """
    return max(restitution1, restitution2)

class ManifoldPoint(object):
    """
    A manifold point is a contact point belonging to a contact
    manifold. It holds details related to the geometry and dynamics
    of the contact points.
    The local point usage depends on the manifold type:
    -CIRCLES: the local center of circleB
    -FACE_A: the local center of cirlceB or the clip point of polygonB
    -FACE_B: the clip point of polygonA
    This structure is stored across time steps, so we keep it small.
    Note: the impulses are used for internal caching and may not
    provide reliable contact forces, especially for high speed collisions.
    """
    __slots__=['local_point', 'normal_impulse', 'tangent_impulse', 'id']
    VERTEX=0
    FACE=1
    def __init__(self, local_point=(0, 0), normal_impulse=0.0, tangent_impulse=0.0, id=None):
        self.local_point=Vec2(*local_point)
        self.normal_impulse=normal_impulse
        self.tangent_impulse=tangent_impulse
        if id is None:
            self.id = (0,0,0,0)
        else:
            self.id = tuple(id)

    def set_id(self, index_a, index_b, type_a, type_b):
        self.id = (index_a, index_b, type_a, type_b)

    def __repr__(self):
        return 'ManifoldPoint(local_point=%s, normal_impulse=%g, tangent_impulse=%g, id=%s)' % (
                        self.local_point, self.normal_impulse, self.tangent_impulse, self.id)

    def __copy__(self):
        return ManifoldPoint(self.local_point, self.normal_impulse, self.tangent_impulse, self.id)

    @property
    def index_a(self):
        return self.id[0]

    @property
    def index_b(self):
        return self.id[1]

    @property
    def type_a(self):
        return self.id[2]

    @property
    def type_b(self):
        return self.id[3]

class ClipVertex(object):
    """
    Used for computing contact manifolds.
    """
    __slots__=['v', 'index_a', 'index_b', 'type_a', 'type_b']
    def __init__(self, vertex, index_a, index_b, type_a, type_b):
        self.v = Vec2(*vertex)
        self.index_a = index_a
        self.index_b = index_b
        self.type_a = type_a
        self.type_b = type_b
   
    def __copy__(self):
        return ClipVertex(self.v, self.index_a, self.index_b, self.type_a, self.type_b)

    @property
    def id(self):
        return (self.index_a, self.index_b, self.type_a, self.type_b)

class Manifold(object):
    """
    A manifold for two touching convex shapes.
    Box2D supports multiple types of contact:
    - clip point versus plane with radius
    - point versus point with radius (circles)
    The local point usage depends on the manifold type:
    -CIRCLES: the local center of circleA
    -FACE_A: the center of faceA
    -FACE_B: the center of faceB

    Similarly the local normal usage:
    -CIRCLES: not used
    -FACE_A: the normal on polygonA
    -FACE_B: the normal on polygonB
    We store contacts in this way so that position correction can
    account for movement, which is critical for continuous physics.
    All contact scenarios must be expressed in one of these types.
    This structure is stored across time steps, so we keep it small.
    """
    __slots__=['points', 'local_normal', 'local_point', 'type', 'point_count']
    CIRCLES=0
    FACE_A=1
    FACE_B=2

    NULL_STATE = 0     # the point doesn't exist
    ADD_STATE = 1      # the point was added in the update
    PERSIST_STATE = 2  # the point persisted across the update
    REMOVE_STATE = 3   # the point was removed in the update
    def __init__(self, points=None, local_normal=(0, 0), local_point=(0, 0), type=CIRCLES, point_count=0):
        if points is None:
            self.points=[ManifoldPoint() for i in range(MAX_MANIFOLD_POINTS)]
        else:
            self.points=[copy(point) for point in points]

        self.local_normal=Vec2(*local_normal)
        self.local_point=Vec2(*local_point)
        self.type=type
        self.point_count=point_count

    def __copy__(self):
        return Manifold(self.points, self.local_normal, self.local_point, self.type, self.point_count)
       
    @property
    def used_points(self):
        return self.points[:self.point_count]

    def get_point_states(self, manifold2):
        """
        Compute the point states given another manifold. The states pertain to the transition from this one
        to manifold2. So state1 is either persist or remove while state2 is either add or persist.
        """
        state1 = []
        state2 = []

        # Detect persists and removes.
        mp1_ids = [mp1.id for mp1 in self.used_points]
        mp2_ids = [mp2.id for mp2 in manifold2.used_points]

        for id1 in mp1_ids:
            if id1 in mp2_ids:
                state1.append(Manifold.PERSIST_STATE)
            else:
                state1.append(Manifold.REMOVE_STATE)

        # Detect persists and adds.

        for id2 in mp2_ids:
            if id2 in mp1_ids:
                state2.append(Manifold.PERSIST_STATE)
            else:
                state2.append(Manifold.ADD_STATE)

        return state1, state2

class WorldManifold(object):
    """This is used to compute the current state of a contact manifold."""
    __slots__ = ['normal', 'points']
    def __init__(self, manifold, xf_a, radius_a, xf_b, radius_b):
        """
        Evaluate the manifold with supplied transforms. This assumes
        modest motion from the original state. This does not change the
        point count, impulses, etc. The radii must come from the shapes
        that generated the manifold.
        """
        self.normal=Vec2()
        self.points=[]

        if manifold.point_count == 0:
            return

        if manifold.type == Manifold.CIRCLES:
            normal=Vec2(1.0, 0.0)
            point_a = xf_a * manifold.local_point
            point_b = xf_b * manifold.points[0].local_point
            if distance_squared(point_a, point_b) > EPSILON_SQR:
                normal = (point_b - point_a).normalized

            c_a = point_a + radius_a * normal
            c_b = point_b - radius_b * normal
            self.points = [0.5 * (c_a + c_b)]

        elif manifold.type == Manifold.FACE_A:
            normal = xf_a._rotation * manifold.local_normal
            plane_point = xf_a * manifold.local_point
           
            for i, mp in enumerate(manifold.used_points):
                clip_point = xf_b * mp.local_point
                c_a = clip_point + (radius_a - (clip_point - plane_point).dot(normal)) * normal
                c_b = clip_point - radius_b * normal
                self.points.append(0.5 * (c_a + c_b))

        elif manifold.type == Manifold.FACE_B:
            normal = xf_b._rotation * manifold.local_normal
            plane_point = xf_b * manifold.local_point

            for i, mp in enumerate(manifold.used_points):
                clip_point = xf_a * mp.local_point
                c_b = clip_point + (radius_b - (clip_point - plane_point).dot(normal)) * normal
                c_a = clip_point - radius_a * normal
                self.points.append(0.5 * (c_a + c_b))

            # Ensure normal points from A to B.
            normal = -normal

        self.normal = normal

class ContactRegister(object):
    __slots__ = ['evaluate_fcn', 'primary']
    def __init__(self):
        self.evaluate_fcn = None
        self.primary = None

class VelocityConstraintPoint(object):
    __slots__ = ['r', 'normal_impulse', 'tangent_impulse', 'normal_mass', 'tangent_mass', 'velocity_bias']

    def __init__(self, step=None, normal_impulse=0.0, tangent_impulse=0.0,
                 ra=(0,0), rb=(0,0), normal_mass=0.0, tangent_mass=0.0, 
                 velocity_bias=0.0):

        if step is not None and step.warm_starting:
            self.normal_impulse = step.dt_ratio * normal_impulse
            self.tangent_impulse = step.dt_ratio * tangent_impulse
        else:
            self.normal_impulse = normal_impulse
            self.tangent_impulse = tangent_impulse

        self.r = (Vec2(*ra), Vec2(*rb))
        self.normal_mass = normal_mass
        self.tangent_mass = tangent_mass
        self.velocity_bias = velocity_bias

    def __repr__(self):
        return 'VelocityConstraintPoint(r=%s, normal_impulse=%g, tangent_impulse=%g, normal_mass=%g, tangent_mass=%g, velocity_bias=%g)' % \
            (self.r,
            self.normal_impulse, self.tangent_impulse, 
            self.normal_mass, self.tangent_mass, 
            self.velocity_bias, )

class ContactVelocityConstraint(object):
    __slots__ = ['points', 'normal', 'normal_mass', 'K', 'indices',
                 'inv_mass', 'inv_i', 'friction', 'restitution', 'contact',
                 'manifold']

    def __init__(self, contact, body_a, body_b, K=None, normal_mass=None):
        self.manifold = contact.manifold
        self.contact = contact
        self.friction = contact._friction
        self.restitution = contact._restitution

        self.indices = (body_a._island_index, body_b._island_index)
        self.inv_mass = (body_a._inv_mass, body_b._inv_mass)
        self.inv_i = (body_a._invI, body_b._invI)

        if normal_mass is not None:
            self.normal_mass = Mat22(*normal_mass)
        else:
            self.normal_mass = Mat22()

        if K is not None:
            self.K = Mat22(*K)
        else:
            self.K = Mat22()

class ContactPositionConstraint(object):
    __slots__ = ['local_points', 'local_normal', 'local_point',
                'indices', 'inv_mass', 
                'local_centers', 'inv_i',
                'type', 'radii']

    def __init__(self, body_a, body_b, radius_a, radius_b, manifold):
        self.indices = (body_a._island_index, body_b._island_index)
        self.inv_mass = (body_a._inv_mass, body_b._inv_mass)
        self.local_centers = (body_a._sweep.local_center, body_b._sweep.local_center)
        self.inv_i = (body_a._invI, body_b._invI)

        self.radii = (radius_a, radius_b)
        self.local_normal = Vec2(*manifold.local_normal)
        self.local_point = Vec2(*manifold.local_point)
        self.type = manifold.type

        self.local_points = []

    def solver_manifold(self, xf_a, xf_b, index):
        """
        Constraint cc, ContactConstraintPoint ccp
        returns: normal, point, separation
        """
        type_ = self.type
        radius_a, radius_b = self.radii
        if type_ == Manifold.CIRCLES:
            point_a = xf_a * self.local_point
            point_b = xf_b * self.local_points[0]
            normal = (point_b - point_a).normalized

            point = 0.5 * (point_a + point_b)
            separation = (point_b - point_a).dot(normal) - radius_a - radius_b

        elif type_ == Manifold.FACE_A:
            normal = xf_a.rotation * self.local_normal
            plane_point = xf_a * self.local_point

            clip_point = xf_b * self.local_points[index]
            separation = (clip_point - plane_point).dot(normal) - radius_a - radius_b
            point = clip_point

        elif type_ == Manifold.FACE_B:
            normal = xf_b.rotation * self.local_normal
            plane_point = xf_b * self.local_point

            clip_point = xf_a * self.local_points[index]
            separation = (clip_point - plane_point).dot(normal) - radius_a - radius_b
            point = clip_point

            # Ensure normal points from A to B
            normal = -normal

        else:
            assert(False)

        return normal, point, separation
