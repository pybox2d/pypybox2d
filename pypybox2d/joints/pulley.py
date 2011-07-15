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

__all__ = ('PulleyJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from ..common import (Vec2, Mat22, scalar_cross, property)
from ..settings import (EPSILON, LINEAR_SLOP)
from .joint import Joint

class PulleyJoint(Joint):
    """
    The pulley joint is connected to two bodies and two fixed ground points.
    The pulley supports a ratio such that:
    length1 + ratio * length2 <= constant
    Yes, the force transmitted is scaled by the ratio.
    Warning: the pulley joint can get a bit squirrelly by itself. They often
    work better when combined with prismatic joints. You should also cover the
    the anchor points with static shapes to prevent one side from going to
    zero length.

    Creation requires two ground anchors, two dynamic body anchor points, 
    and a pulley ratio.
    """
    # Pulley:
    # length1 = norm(p1 - s1)
    # length2 = norm(p2 - s2)
    # C0 = (length1 + ratio * length2)_initial
    # C = C0 - (length1 + ratio * length2)
    # u1 = (p1 - s1) / norm(p1 - s1)
    # u2 = (p2 - s2) / norm(p2 - s2)
    # Cdot = -dot(u1, va + cross(wa, ra)) - ratio * dot(u2, vb + cross(wb, rb))
    # J = -[u1 cross(ra, u1) ratio * u2  ratio * cross(rb, u2)]
    # K = J * invM * JT
    #   = invMass1 + invIa * cross(ra, u1)^2 + ratio^2 * (invMass2 + invIb * cross(rb, u2)^2)
    def __init__(self, body_a, body_b, 
                 ground_anchor_a=(0, 0), ground_anchor_b=(0, 0), 
                 anchor_a=(0, 0), anchor_b=(0, 0), ratio=1.0,
                 length_a=None, length_b=None, 
                 collide_connected=False, local_anchor_a=None, local_anchor_b=None):
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        if ratio == 0.0 or ratio < EPSILON:
            raise ValueError('Ratio must be positive, non-zero and not too small')

        if local_anchor_a is None:
            self._local_anchor_a = body_a.get_local_point(anchor_a)
        else:
            self._local_anchor_a = Vec2(*local_anchor_a)

        if local_anchor_b is None:
            self._local_anchor_b = body_b.get_local_point(anchor_b)
        else:
            self._local_anchor_b = Vec2(*local_anchor_b)

        self._ground_anchor_a = Vec2(*ground_anchor_a)
        self._ground_anchor_b = Vec2(*ground_anchor_b)

        self._u1 = Vec2()
        self._u2 = Vec2()
       
        if length_a is None:
            length_a = (anchor_a - self._ground_anchor_a).length
        if length_b is None:
            length_b = (anchor_b - self._ground_anchor_b).length

        self._length_a = length_a
        self._length_b = length_b
        self._constant = length_a + ratio * length_b
        self._ratio = ratio
        
        # Effective masses
        self._mass = 0.0

        # Impulses for accumulation/warm starting.
        self._impulse = 0.0

    def __copy__(self):
        return PulleyJoint(self._body_a, self._body_b, 
                           self._ground_anchor_a, self._ground_anchor_b,
                           None, None, self._ratio,
                           self._length_a, self._length_b,
                           self._collide_connected, 
                           self._local_anchor_a, self._local_anchor_b)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * self._impulse * self._u2

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return 0.0

    @property
    def length_a(self):
        """The current length of the segment attached to body_a"""
        d = self.anchor_a - self._ground_anchor_a
        return d.length

    @property
    def length_b(self):
        """The current length of the segment attached to body_b"""
        d = self.anchor_b - self._ground_anchor_b
        return d.length

    @property
    def ground_anchor_a(self):
        """The first ground anchor. This point never moves."""
        return copy(self._ground_anchor_a)

    @property
    def ground_anchor_b(self):
        """The second ground anchor. This point never moves."""
        return copy(self._ground_anchor_b)

    @property
    def anchor_a(self):
        """The world anchor point relative to body_a's origin."""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """The world anchor point relative to body_b's origin."""
        return self._body_b.get_world_point(self._local_anchor_b)

    @property
    def ratio(self):
        """The pulley ratio, used to simulate a block-and-tackle."""
        return self._ratio

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

        # Get the pulley axes.
        ua = ca + ra - self._ground_anchor_a
        ub = cb + rb - self._ground_anchor_b

        length_a = ua.length
        length_b = ub.length

        if length_a > 10.0 * LINEAR_SLOP:
            self._ua = ua / length_a
        else:
            self._ua = Vec2()

        if length_b > 10.0 * LINEAR_SLOP:
            self._ub = ub / length_b
        else:
            self._ub = Vec2()

        # Compute effective mass.
        rua = ra.cross(self._ua)
        rub = rb.cross(self._ub)

        ma = ma + ia * rua ** 2
        mb = mb + ib * rub ** 2

        self._mass = ma + self._ratio * self._ratio * mb

        if self._mass > 0.0:
            self._mass = 1.0 / self._mass

        if step.warm_starting:
            # Scale impulses to support variable time steps.
            self._impulse *= step.dt_ratio

            # Warm starting.
            Pa = -self._impulse * self._ua
            Pb = (-self._ratio * self._impulse) * self._ub
            va += ma * Pa
            wa += ia * ra.cross(Pa)
            vb += mb * Pb
            wb += ib * rb.cross(Pb)
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
        ua, ub = self._ua, self._ub

        vpa = va + scalar_cross(wa, ra)
        vpb = vb + scalar_cross(wb, rb)

        Cdot = -ua.dot(vpa) - self._ratio * ub.dot(vpb)
        impulse = -self._mass * Cdot
        self._impulse += impulse

        Pa = -impulse * ua
        Pb = -self._ratio * impulse * ub
        va += ma * Pa
        wa += ia * ra.cross(Pa)
        vb += mb * Pb
        wb += ib * rb.cross(Pb)

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

        # Get the pulley axes.
        ua = ca + ra - self._ground_anchor_a
        ub = cb + rb - self._ground_anchor_b

        length_a = ua.length
        length_b = ub.length

        if length_a > 10.0 * LINEAR_SLOP:
            ua = ua / length_a
        else:
            ua = Vec2()

        if length_b > 10.0 * LINEAR_SLOP:
            ub = ub / length_b
        else:
            ub = Vec2()

        # Compute effective mass.
        rua = ra.cross(ua)
        rub = rb.cross(ub)

        ma = ma + ia * rua ** 2
        mb = mb + ib * rub ** 2
        mass = ma + self._ratio ** 2 * mb

        if mass > 0.0:
            mass = 1.0 / mass

        C = self._constant - length_a - self._ratio * length_b
        linear_error = abs(C)

        impulse = -mass * C

        Pa = -impulse * ua
        Pb = -self._ratio * impulse * ub

        ca += ma * Pa
        aa += ia * ra.cross(Pa)
        cb += mb * Pb
        ab += ib * rb.cross(Pb)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

        return linear_error < LINEAR_SLOP


