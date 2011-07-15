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

__all__ = ('RopeJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Mat22, scalar_cross, clamp, property)
from ..settings import (LINEAR_SLOP, MAX_LINEAR_CORRECTION)
from .joint import (INACTIVE_LIMIT, AT_UPPER_LIMIT)
from .joint import Joint

class RopeJoint(Joint):
    """
    A rope joint enforces a maximum distance between two points
    on two bodies. It has no other effect.

    Warning: if you attempt to change the maximum length during
    the simulation you will get some non-physical behavior.
    A model that would allow you to dynamically modify the length
    would have some sponginess, so I chose not to implement it
    that way. See DistanceJoint if you want to dynamically
    control length.

    Creation requires two body anchor points and a maximum lengths.
    Note: by default the connected objects will not collide,
    set collide_connected to True if you want them to collide.
    
    """
    # Limit:
    # C = norm(pB - pA) - L
    # u = (pB - pA) / norm(pB - pA)
    # Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    # J = [-u -cross(rA, u) u cross(rB, u)]
    # K = J * invM * JT
    #   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2
    def __init__(self, body_a, body_b, local_anchor_a=(0, 0), local_anchor_b=(0, 0),
                 max_length=0.0, collide_connected=False):

        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        self._local_anchor_a = Vec2(*local_anchor_a)
        self._local_anchor_b = Vec2(*local_anchor_a)
        self._state = INACTIVE_LIMIT
        self._length = 0.0
        self._max_length = max_length

        # Jacobian info
        self._u = Vec2()
        self._ra = Vec2() 
        self._rb = Vec2()

        # Effective mass
        self._mass = 0.0

        # Impulses for accumulation/warm starting
        self._impulse = 0.0

    def __copy__(self):
        return RopeJoint(self._body_a, self._body_b, self._local_anchor_a,
                         self._local_anchor_b, self._max_length, 
                         self._collide_connected)


    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return (inv_dt * self._impulse) * self._u

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return 0.0

    @property
    def limit_state(self):
        """The status of the limit (INACTIVE_LIMIT, AT_UPPER_LIMIT)"""
        return self._limit_state

    @property
    def max_length(self):
        """(Read-only) Maximum separation/rope length"""
        return self._max_length

    def _init_velocity_constraints(self, step, positions, velocities):
        body_a, body_b = self._body_a, self._body_b
        index_a = body_a._island_index
        index_b = body_b._island_index
        self._indices = (index_a, index_b)

        local_center_a = self._local_center_a = body_a._sweep.local_center
        local_center_b = self._local_center_b = body_b._sweep.local_center

        ma = self._inv_mass_a = body_a._inv_mass
        mb = self._inv_mass_b = body_b._inv_mass

        ia = self._inv_Ia = body_a._invI
        ib = self._inv_Ib = body_b._invI

        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute the effective mass matrix.
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        # Rope axis
        u = self._u = cb + rb - ca - ra

        self._length = u.length

        C = self._length - self._max_length
        if C > 0.0:
            self._state = AT_UPPER_LIMIT
        else:
            self._state = INACTIVE_LIMIT

        if self._length > LINEAR_SLOP:
            u /= self._length
        else:
            u = Vec2()
            self._mass = 0.0
            self._impulse = 0.0
            return

        # Compute effective mass.
        cra = ra.cross(u)
        crb = rb.cross(u)
        inv_mass = ma + ia * cra * cra + mb + ib * crb * crb

        if inv_mass != 0.0:
            self._mass = 1.0 / inv_mass
        else:
            self._mass = 0.0

        if step.warm_starting:
            # Scale the impulse to support a variable time step.
            self._impulse *= step.dt_ratio

            P = self._impulse * self._u
            va -= ma * P
            wa -= ia * self._ra.cross(P)
            vb += mb * P
            wb += ib * self._rb.cross(P)
        else:
            self._impulse = 0.0

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        ra, rb = self._ra, self._rb
        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Cdot = dot(u, v + cross(w, r))
        vpa = va + scalar_cross(wa, ra)
        vpb = vb + scalar_cross(wb, rb)
        C = self._length - self._max_length
        Cdot = self._u.dot(vpb - vpa)

        # Predictive constraint.
        if C < 0.0:
            Cdot += step.inv_dt * C

        impulse = -self._mass * Cdot
        old_impulse = self._impulse
        self._impulse = min(0.0, self._impulse + impulse)
        impulse = self._impulse - old_impulse

        P = impulse * self._u
        va -= ma * P
        wa -= ia * self._ra.cross(P)
        vb += mb * P
        wb += ib * self._rb.cross(P)

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)
    
    def _solve_position_constraints(self, step, positions, velocities):
        """This returns true if the position errors are within tolerance."""
        index_a, index_b = self._indices
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        u = cb + rb - ca - ra

        length = u.length
        u = u.normalized
        C = length - self._max_length

        C = clamp(C, 0.0, MAX_LINEAR_CORRECTION)

        impulse = -self._mass * C
        P = impulse * u

        ca -= ma * P
        aa -= ia * ra.cross(P)
        cb += mb * P
        ab += ib * rb.cross(P)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)
        return length - self._max_length < LINEAR_SLOP


