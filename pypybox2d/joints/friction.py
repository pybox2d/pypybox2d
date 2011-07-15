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

__all__ = ('FrictionJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Mat22, scalar_cross, clamp, is_valid_float, property)
from .joint import Joint

class FrictionJoint(Joint):
    """
    Friction joint. This is used for top-down friction.
    It provides 2D translational friction and angular friction.
    """
    # Point-to-point constraint
    # Cdot = vb - va
    #      = vb + cross(wb, rb) - va - cross(wa, ra)
    # J = [-I -ra_skew I rb_skew ]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)

    # Angle constraint
    # Cdot = wb - wa
    # J = [0 0 -1 0 0 1]
    # K = invIa + invIb
    def __init__(self, body_a, body_b, local_anchor_a=(0,0), local_anchor_b=(0,0),
                         max_force=0.0, max_torque=0.0, collide_connected=False):
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        self._local_anchor_a = Vec2(*local_anchor_a)
        self._local_anchor_b = Vec2(*local_anchor_b)
        self._linear_impulse = Vec2()
        self._angular_impulse = 0.0
        self._max_force = max_force
        self._max_torque = max_torque

    def __copy__(self):
        return FrictionJoint(self._body_a, self._body_b, self._local_anchor_a, 
                            self._local_anchor_b, self._max_force, self._max_torque, 
                            self._collide_connected)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * self._linear_impulse

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._angular_impulse

    @property
    def anchor_a(self):
        """Get the anchor point on body_a in world coordinates"""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """Get the anchor point on body_b in world coordinates"""
        return self._body_b.get_world_point(self._local_anchor_b)

    @property
    def max_force(self):
        """The maximum friction force in N."""
        return self._max_force

    @max_force.setter
    def max_force(self, max_force):
        if not is_valid_float(max_force) or max_force < 0.0:
            raise ValueError('Max force must be >= 0.0')

        self._max_force = max_force

    @property
    def max_torque(self):
        """The maximum friction torque in N*m."""
        return self._max_torque

    @max_torque.setter
    def max_torque(self, max_torque):
        if not is_valid_float(max_torque) or max_torque < 0.0:
            raise ValueError('Max torque must be >= 0.0')
        self._max_torque = max_torque

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

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        # J = [-I -ra_skew I rb_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ mA+ray^2*iA+mB+rby^2*iB,  -ray*iA*rax-rby*iB*rbx,          -ray*iA-rby*iB]
        #     [  -ray*iA*rax-rby*iB*rbx, mA+rax^2*iA+mB+rbx^2*iB,           rax*iA+rbx*iB]
        #     [          -ray*iA-rby*iB,           rax*iA+rbx*iB,                   iA+iB]

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        K = Mat22()
        K.col1 = Vec2(inv_mass_a + inv_mass_b + inv_Ia * ra.y * ra.y + inv_Ib * rb.y * rb.y,
                      -inv_Ia * ra.x * ra.y - inv_Ib * rb.x * rb.y)
        K.col2 = Vec2(K.col1.y,
                      inv_mass_a + inv_mass_b + inv_Ia * ra.x * ra.x + inv_Ib * rb.x * rb.x)

        self._linear_mass = K.inverse
        self._angular_mass = inv_Ia + inv_Ib

        if self._angular_mass > 0.0:
            self._angular_mass = 1.0 / self._angular_mass

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._linear_impulse *= step.dt_ratio
            self._angular_impulse *= step.dt_ratio
            
            P = self._linear_impulse
            va -= inv_mass_a * P
            wa -= inv_Ia * (ra.cross(P) + self._angular_impulse)

            vb += inv_mass_b * P
            wb += inv_Ib * (rb.cross(P) + self._angular_impulse)
        else:
            self._linear_impulse = Vec2()
            self._angular_impulse = 0.0

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        dt = step.dt

        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        ra, rb = self._ra, self._rb

        inv_mass_a, inv_mass_b = self._inv_mass_a, self._inv_mass_b
        inv_Ia, inv_Ib = self._inv_Ia, self._inv_Ib

        # Solve angular friction
        Cdot = wb - wa
        impulse = -self._angular_mass * Cdot

        old_impulse = self._angular_impulse
        max_impulse = dt * self._max_torque
        self._angular_impulse = clamp(self._angular_impulse + impulse, -max_impulse, max_impulse)
        impulse = self._angular_impulse - old_impulse

        wa -= inv_Ia * impulse
        wb += inv_Ib * impulse

        # Solve linear friction
        Cdot = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)

        impulse = -(self._linear_mass * Cdot)
        old_impulse = Vec2(*self._linear_impulse)
        self._linear_impulse += impulse

        max_impulse = dt * self._max_force

        if self._linear_impulse.length_squared > max_impulse ** 2:
            self._linear_impulse = self._linear_impulse.normalized * max_impulse

        impulse = self._linear_impulse - old_impulse

        va -= inv_mass_a * impulse
        wa -= inv_Ia * ra.cross(impulse)

        vb += inv_mass_b * impulse
        wb += inv_Ib * rb.cross(impulse)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

    def _solve_position_constraints(self, step, positions, velocities):
        return True

