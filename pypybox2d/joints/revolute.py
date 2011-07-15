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

__all__ = ('RevoluteJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Vec3, Mat22, Mat33, scalar_cross, clamp, property)
from ..settings import (LINEAR_SLOP, ANGULAR_SLOP, MAX_ANGULAR_CORRECTION)
from .joint import (INACTIVE_LIMIT, AT_LOWER_LIMIT, AT_UPPER_LIMIT, EQUAL_LIMITS)
from .joint import Joint

class RevoluteJoint(Joint):
    """
    A revolute joint constrains two bodies to share a common point while they
    are free to rotate about the point. The relative rotation about the shared
    point is the joint angle. You can limit the relative rotation with
    a joint limit that specifies a lower and upper angle. You can use a motor
    to drive the relative rotation about the shared point. A maximum motor torque
    is provided so that infinite forces are not generated.

    Revolute joint creation requires defining an
    anchor point where the bodies are joined. The definition
    uses local anchor points so that the initial configuration
    can violate the constraint slightly. You also need to
    specify the initial relative angle for joint limits. This
    helps when saving and loading a game.
    The local anchor points are measured from the body's origin
    rather than the center of mass because:
    1. you might not know where the center of mass will be.
    2. if you add/remove shapes from a body and recompute the mass,
       the joints will be broken.
    """

    def __init__(self, body_a, body_b, anchor=(0,0), reference_angle=None, 
                    lower_angle=0.0, upper_angle=0.0, max_motor_torque=0.0,
                    motor_speed=0.0, limit_enabled=False, motor_enabled=False,
                    local_anchor_a=None, local_anchor_b=None,
                    collide_connected=False):
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        if local_anchor_a is not None:
            self._local_anchor_a = Vec2(*local_anchor_a)
        else:
            self._local_anchor_a = body_a.get_local_point(anchor)

        if local_anchor_b is not None:
            self._local_anchor_b = Vec2(*local_anchor_b)
        else:
            self._local_anchor_b = body_b.get_local_point(anchor)

        if reference_angle is not None:
            self._reference_angle = float(reference_angle)
        else:
            self._reference_angle = body_b.angle - body_a.angle

        self._impulse = Vec3()
        self._lower_angle = float(lower_angle)
        self._upper_angle = float(upper_angle)
        self._max_motor_torque = float(max_motor_torque)
        self._limit_enabled = bool(limit_enabled)
        self._motor_enabled = bool(motor_enabled)
        self._motor_impulse = 0.0
        self._motor_speed = float(motor_speed)
        self._limit_state = INACTIVE_LIMIT
        self._mass = Mat33()

    def __copy__(self):
        return RevoluteJoint(self.body_a, self.body_b, None,
                             self._reference_angle, self._lower_angle,
                             self._upper_angle, self._max_motor_torque,
                             self._motor_seeed, self._limit_enabled,
                             self._motor_enabled, self._local_anchor_a,
                             self._local_anchor_b, self._collide_connected)

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
    def joint_angle(self):
        """The current joint angle in radians."""
        return self._body_b._sweep.a - self._body_a._sweep.a - self._reference_angle

    @property
    def joint_speed(self):
        """The current joint speed in radians/sec."""
        return self._body_b._angular_velocity - self._body_a._angular_velocity

    @property
    def motor_speed(self):
        """The current motor speed in radians/sec."""
        return self._motor_speed

    @motor_speed.setter
    def motor_speed(self, speed):
        """The current motor speed in radians/sec."""
        for body in self.bodies:
            body.awake = True
        self._motor_speed = speed

    @property
    def limit_enabled(self):
        """The joint limit (min/max angle) enabled"""
        return self._limit_enabled

    @limit_enabled.setter
    def limit_enabled(self, limit_enabled):
        if limit_enabled != self._limit_enabled:
            for body in self.bodies:
                body.awake = True
            self._limit_enabled = limit_enabled
            self._impulse = Vec3(self._impulse.x, self._impulse.y, 0.0)

    @property
    def motor_enabled(self):
        """Joint motor enabled"""
        return self._motor_enabled

    @motor_enabled.setter
    def motor_enabled(self, motor_enabled):
        if motor_enabled != self._motor_enabled:
            self._motor_enabled = motor_enabled
            for body in self.bodies:
                body.awake = True

    @property
    def lower_angle(self):
        """The lower angular limit"""
        return self._lower_angle

    @lower_angle.setter
    def lower_angle(self, lower_angle):
        if self._lower_angle != lower_angle:
            for body in self.bodies:
                body.awake = True
            self._impulse = Vec3(self._impulse.x, self._impulse.y, 0.0)
            self._lower_angle = lower_angle

    @property
    def upper_angle(self):
        """The upper angular limit"""
        return self._upper_angle

    @upper_angle.setter
    def upper_angle(self, upper_angle):
        if self._upper_angle != upper_angle:
            for body in self.bodies:
                body.awake = True
            self._impulse = Vec3(self._impulse.x, self._impulse.y, 0.0)
            self._upper_angle = upper_angle

    @property
    def max_motor_torque(self):
        """The maximum motor torque, usually in N-m"""
        return self._max_motor_torque

    @max_motor_torque.setter
    def max_motor_torque(self, torque):
        for body in self.bodies:
            body.awake = True
        self._max_motor_torque = torque

    def get_motor_torque(self, inv_dt):
        return inv_dt * self._motor_impulse

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * Vec2(*self._impulse[:2])

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._impulse.z

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

        # J = [-I -ra_skew I rb_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ mA+ray^2*iA+mB+rby^2*iB,  -ray*iA*rax-rby*iB*rbx,          -ray*iA-rby*iB]
        #     [  -ray*iA*rax-rby*iB*rbx, mA+rax^2*iA+mB+rbx^2*iB,           rax*iA+rbx*iB]
        #     [          -ray*iA-rby*iB,           rax*iA+rbx*iB,                   iA+iB]

        self._mass.col3 = Vec3(-ra.y * ia - rb.y * ib,
                                ra.x * ia + rb.x * ib,
                                ia + ib)
        self._mass.col2 = Vec3(-ra.y * ra.x * ia - rb.y * rb.x * ib,
                                ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib,
                                self._mass.col3.y)
        self._mass.col1 = Vec3(ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib,
                               self._mass.col2.x,
                               self._mass.col3.x)

        self._motor_mass = ia + ib
        fixed_rotation = (self._motor_mass == 0.0)

        if self._motor_mass > 0.0:
            self._motor_mass = 1.0 / self._motor_mass

        if not self._motor_enabled or fixed_rotation:
            self._motor_impulse = 0.0

        if self._limit_enabled and not fixed_rotation:
            joint_angle = ab - aa - self._reference_angle
            if abs(self._upper_angle - self._lower_angle) < 2.0 * ANGULAR_SLOP:
                self._limit_state = EQUAL_LIMITS
            elif joint_angle <= self._lower_angle:
                if self._limit_state != AT_LOWER_LIMIT:
                    self._impulse -= (0.0, 0.0, self._impulse.z)
                self._limit_state = AT_LOWER_LIMIT
            elif joint_angle >= self._upper_angle:
                if self._limit_state != AT_UPPER_LIMIT:
                    self._impulse -= (0.0, 0.0, self._impulse.z)
                self._limit_state = AT_UPPER_LIMIT
            else:
                self._limit_state = INACTIVE_LIMIT
                self._impulse -= (0.0, 0.0, self._impulse.z)
        else:
            self._limit_state = INACTIVE_LIMIT

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._impulse *= step.dt_ratio
            self._motor_impulse *= step.dt_ratio

            P = Vec2(self._impulse.x, self._impulse.y)

            va -= ma * P
            wa -= ia * (ra.cross(P) + self._motor_impulse + self._impulse.z)

            vb += mb * P
            wb += ib * (rb.cross(P) + self._motor_impulse + self._impulse.z)
        else:
            self._impulse = Vec3()
            self._motor_impulse = 0.0

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib
        ra, rb = self._ra, self._rb

        fixed_rotation = ((ia + ib) == 0.0)

        # Solve motor constraint.
        if self._motor_enabled and self._limit_state != EQUAL_LIMITS and not fixed_rotation:
            Cdot = wb - wa - self._motor_speed
            impulse = self._motor_mass * (-Cdot)
            old_impulse = self._motor_impulse
            max_impulse = step.dt * self._max_motor_torque
            self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
            impulse = self._motor_impulse - old_impulse

            wa -= ia * impulse
            wb += ib * impulse

        # Solve limit constraint.
        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT and not fixed_rotation:
            # Solve point-to-point constraint
            Cdot1 = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)
            Cdot2 = wb - wa
            Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

            impulse = -self._mass.solve3x3(Cdot)

            if self._limit_state == EQUAL_LIMITS:
                self._impulse += impulse
            elif self._limit_state == AT_LOWER_LIMIT:
                new_impulse = self._impulse.z + impulse.z
                if new_impulse < 0.0:
                    rhs = -Cdot1 + self._impulse.z * Vec2(self._mass.col3.x, self._mass.col3.y)
                    reduced = self._mass.solve2x2(rhs)
                    impulse = Vec3(reduced.x, reduced.y, -self._impulse.z)
                    self._impulse += (reduced.x, reduced.y, -self._impulse.z)
                else:
                    self._impulse += impulse
            elif self._limit_state == AT_UPPER_LIMIT:
                new_impulse = self._impulse.z + impulse.z
                if new_impulse > 0.0:
                    rhs = -Cdot1 + self._impulse.z * Vec2(self._mass.col3.x, self._mass.col3.y)
                    reduced = self._mass.solve2x2(rhs)
                    impulse = Vec3(reduced.x, reduced.y, -self._impulse.z)
                    self._impulse += (reduced.x, reduced.y, -self._impulse.z)
                else:
                    self._impulse += impulse

            P = Vec2(impulse.x, impulse.y)

            va -= ma * P
            wa -= ia * (ra.cross(P) + impulse.z)

            vb += mb * P
            wb += ib * (rb.cross(P) + impulse.z)
        else:
            # Solve point-to-point constraint
            Cdot = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)
            impulse = self._mass.solve2x2(-Cdot)

            self._impulse += (impulse.x, impulse.y, 0.0)

            va -= ma * impulse
            wa -= ia * ra.cross(impulse)

            vb += mb * impulse
            wb += ib * rb.cross(impulse)

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_position_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib
        fixed_rotation = ((ia + ib) == 0.0)
        angular_error = 0.0
        position_error = 0.0

        # Solve angular limit constraint.
        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT and not fixed_rotation:
            angle = ab - aa - self._reference_angle
            limit_impulse = 0.0

            if self._limit_state == EQUAL_LIMITS:
                # Prevent large angular corrections
                C = clamp(angle - self._lower_angle, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION)
                limit_impulse = -self._motor_mass * C
                angular_error = abs(C)
            elif self._limit_state == AT_LOWER_LIMIT:
                C = angle - self._lower_angle
                angular_error = -C

                # Prevent large angular corrections and allow some slop.
                C = clamp(C + ANGULAR_SLOP, -MAX_ANGULAR_CORRECTION, 0.0)
                limit_impulse = -self._motor_mass * C
            elif self._limit_state == AT_UPPER_LIMIT:
                C = angle - self._upper_angle
                angular_error = C

                # Prevent large angular corrections and allow some slop.
                C = clamp(C - ANGULAR_SLOP, 0.0, MAX_ANGULAR_CORRECTION)
                limit_impulse = -self._motor_mass * C

            aa -= ia * limit_impulse
            ab += ib * limit_impulse

        # Solve point-to-point constraint.
        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        C = cb + rb - ca - ra
        position_error = C.length
        
        rax, ray = ra
        rbx, rby = rb
        K = Mat22()
        K.col1 = Vec2(ma + mb + ia * ray * ray + ib * rby * rby,
                      -ia * rax * ray - ib * rbx * rby)
        K.col2 = Vec2(K.col1.y,
                      ma + mb + ia * rax * rax + ib * rbx * rbx)

        impulse = -K.solve(C)

        ca -= ma * impulse
        aa -= ia * ra.cross(impulse)

        cb += mb * impulse
        ab += ib * rb.cross(impulse)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)
        return position_error <= LINEAR_SLOP and angular_error <= ANGULAR_SLOP

