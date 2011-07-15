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

__all__ = ('DistanceJoint', )

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (PI, Vec2, Mat22, scalar_cross, clamp, property)
from ..settings import (LINEAR_SLOP, MAX_LINEAR_CORRECTION)
from .joint import Joint

class DistanceJoint(Joint):
    """
    This requires defining an
    anchor point on both bodies and the non-zero length of the
    distance joint. This uses local anchor points
    so that the initial configuration can violate the constraint
    slightly. This helps when saving and loading a game.
    @warning Do not use a zero or short length.
    """

    # 1-D constrained system
    # m (vb - va) = lambda
    # vb + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
    # x2 = x1 + h * vb
    #                                                                           
    # 1-D mass-damper-spring system
    # m (vb - va) + h * d * vb + h * k * 
    #                                                                           
    # C = norm(p2 - p1) - L
    # u = (p2 - p1) / norm(p2 - p1)
    # Cdot = dot(u, vb + cross(wb, rb) - va - cross(wa, ra))
    # J = [-u -cross(ra, u) u cross(rb, u)]
    # K = J * invM * JT
    #   = invMass1 + invIa * cross(ra, u)^2 + invMass2 + invIb * cross(rb, u)^2
    def __init__(self, body_a, body_b, anchor_a=(0,0), anchor_b=(0,0), frequency=0.0, 
                    damping_ratio=0.0, length=None, collide_connected=False, 
                    local_anchor_a=None, local_anchor_b=None):
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        if local_anchor_a is not None:
            self._local_anchor_a = Vec2(*local_anchor_a)
        else:
            self._local_anchor_a = body_a.get_local_point(anchor_a)

        if local_anchor_b is not None:
            self._local_anchor_b = Vec2(*local_anchor_b)
        else:
            self._local_anchor_b = body_b.get_local_point(anchor_b)

        if length is not None:
            self._length = float(length)
        else:
            d = self.anchor_b - self.anchor_a
            self._length = d.length

        if self._length <= LINEAR_SLOP:
            raise ValueError('length must be > 0 and not a very small number')

        self._frequency = frequency
        self._damping_ratio = damping_ratio
        self._impulse = 0.0
        self._gamma = 0.0
        self._bias = 0.0
        self._u = Vec2()
        self._mass = 0.0

    def __copy__(self):
        return DistanceJoint(self.body_a, self.body_b, self.anchor_a, self.anchor_b, 
                             self._frequency, self._damping_ratio,
                             self._length, self._collide_connected,
                             self._local_anchor_a, self._local_anchor_b)

    @property
    def local_anchor_a(self):
        """The local anchor point relative to body_a's origin."""
        return Vec2(*self._local_anchor_a)

    @property
    def local_anchor_b(self):
        """The local anchor point relative to body_b's origin."""
        return Vec2(*self._local_anchor_b)

    @property
    def anchor_a(self):
        """The world anchor point relative to body_a's origin."""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """The world anchor point relative to body_b's origin."""
        return self._body_b.get_world_point(self._local_anchor_b)

    @property
    def length(self):
        """
        The natural length between the anchor points.

        Manipulating the length can lead to non-physical behavior when the frequency is zero.
        """
        return self._length

    @length.setter
    def length(self, length):
        self._length = length

    @property
    def frequency(self):
        """The mass-spring-damper frequency in Hertz."""
        return self._frequency

    @frequency.setter
    def frequency(self, frequency):
        self._frequency = frequency

    @property
    def damping_ratio(self):
        """The damping ratio. 0 = no damping, 1 = critical damping."""
        return self._damping_ratio

    @damping_ratio.setter
    def damping_ratio(self, damping_ratio):
        self._damping_ratio = damping_ratio

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return (inv_dt * self._impulse) * self._u

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return 0.0

    def _init_velocity_constraints(self, step, positions, velocities):
        body_a, body_b = self._body_a, self._body_b
        index_a = body_a._island_index
        index_b = body_b._island_index
        self._indices = (index_a, index_b)

        local_center_a = self._local_center_a = body_a._sweep.local_center
        local_center_b = self._local_center_b = body_b._sweep.local_center

        inv_mass_a = self._inv_mass_a = body_a._inv_mass
        inv_mass_b = self._inv_mass_b = body_b._inv_mass

        inv_Ia = self._inv_Ia = body_a._invI
        inv_Ib = self._inv_Ib = body_b._invI

        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute the effective mass matrix.
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        u = self._u = cb + rb - ca - ra

        # Handle singularity.
        length = self._u.length
        if length > LINEAR_SLOP:
            self._u *= 1.0 / length
        else:
            self._u = Vec2(0.0, 0.0)

        cr_au = ra.cross(u) ** 2
        cr_bu = rb.cross(u) ** 2
        inv_mass = inv_mass_a + inv_Ia * cr_au + inv_mass_b + inv_Ib * cr_bu

        if inv_mass != 0.0:
            self._mass = 1.0 / inv_mass
        else:
            self._mass = 0.0

        if self._frequency > 0.0:
            C = length - self._length

            # Frequency
            omega = 2.0 * PI * self._frequency

            # Damping coefficient
            d = 2.0 * self._mass * self._damping_ratio * omega

            # Spring stiffness
            k = self._mass * (omega ** 2)

            # magic formulas
            dt = step.dt

            gamma = dt * (d + dt * k)
            if gamma != 0.0:
                self._gamma = 1.0 / gamma
            else:
                self._gamma = 0.0
            self._bias = C * dt * k * self._gamma

            mass = inv_mass + self._gamma
            if mass != 0.0:
                self._mass = 1.0 / mass
            else:
                self._mass = 0.0

        if step.warm_starting:
            # Scale the impulse to support a variable time step.
            self._impulse *= step.dt_ratio

            P = self._impulse * self._u
            va -= inv_mass_a * P
            wa -= inv_Ia * ra.cross(P)
            vb += inv_mass_b * P
            wb += inv_Ib * rb.cross(P)
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
        Cdot = self._u.dot(vpb - vpa)

        impulse = -self._mass * (Cdot + self._bias + self._gamma * self._impulse)
        self._impulse += impulse

        P = impulse * self._u
        va -= ma * P
        wa -= ia * ra.cross(P)
        vb += mb * P
        wb += ib * rb.cross(P)

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_position_constraints(self, step, positions, velocities):
        if self._frequency> 0.0:
            # There is no position correction for soft distance constraints.
            return True

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
        C = length - self._length

        impulse = -self._mass * clamp(C, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION)
        P = impulse * u

        ca -= ma * P
        aa -= ia * ra.cross(P)
        cb += mb * P
        ab += ib * rb.cross(P)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

        return abs(C) < LINEAR_SLOP


