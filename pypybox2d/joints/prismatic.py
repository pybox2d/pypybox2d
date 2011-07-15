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

__all__ = ('PrismaticJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Vec3, Mat22, Mat33, scalar_cross, clamp, property)
from ..settings import (LINEAR_SLOP, ANGULAR_SLOP, MAX_LINEAR_CORRECTION)
from .joint import (INACTIVE_LIMIT, AT_LOWER_LIMIT, AT_UPPER_LIMIT, EQUAL_LIMITS)
from .joint import Joint

class PrismaticJoint(Joint):
    """
    A prismatic joint. This joint provides one degree of freedom: translation
    along an axis fixed in body_a. Relative rotation is prevented. You can
    use a joint limit to restrict the range of motion and a joint motor to
    drive the motion or to model joint friction.
    """
    def __init__(self, body_a, body_b, anchor=(0,0), axis=(0,1), 
                    lower_limit=0.0, upper_limit=0.0, max_motor_force=0.0, 
                    max_motor_torque=0.0, motor_speed=0.0, 
                    reference_angle=None, limit_enabled=False,
                    motor_enabled=False, collide_connected=False,
                    local_anchor_a=None, local_anchor_b=None, local_x_axis=None):
        """
        This requires defining a line of
        motion using an axis and an anchor point. The definition uses local
        anchor points and a local axis so that the initial configuration
        can violate the constraint slightly. The joint translation is zero
        when the local anchor points coincide in world space. Using local
        anchors and a local axis helps when saving and loading a game.
        @warning at least one body should by dynamic with a non-fixed rotation.
        """
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

        if local_x_axis is not None:
            self._local_x_axis = Vec2(*local_x_axis)
        else:
            self._local_x_axis = body_a.get_local_vector(axis)

        self._local_y_axis = scalar_cross(1.0, self._local_x_axis)
        self._motor_mass = 0.0
        self._motor_impulse = 0.0
        self._axis = Vec2()
        self._perp = Vec2()
        self._lower_limit = float(lower_limit)
        self._upper_limit = float(upper_limit)
        self._impulse = Vec3()
        self._max_motor_force = float(max_motor_force)
        self._limit_enabled = bool(limit_enabled)
        self._motor_enabled = bool(motor_enabled)
        self._motor_speed = float(motor_speed)
        self._limit_state = INACTIVE_LIMIT
        self._local_center_a = Vec2()
        self._local_center_b = Vec2()

        self._s1, self._s2 = 0.0, 0.0
        self._a1, self._a2 = 0.0, 0.0

        self._k = None

    def __copy__(self):
        return PrismaticJoint(self._body_a, self._body_b, 
                self._anchor, self._axis, self._lower_limit, self._upper_limit, self._max_motor_force,
                self._max_motor_torque, self._motor_speed, self._reference_angle, self._limit_enabled,
                self._motor_enabled, self._collide_connected, self._local_anchor_a, self._local_anchor_b,
                self._local_x_axis)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * (self._impulse.x * self._perp + (self._motor_impulse + self._impulse.z) * self._axis)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._impulse.y

    @property
    def axis(self):
        return self._body_a.get_world_vector(self._local_x_axis)

    @property
    def anchor_a(self):
        """The world anchor point relative to body_a's origin."""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """The world anchor point relative to body_b's origin."""
        return self._body_b.get_world_point(self._local_anchor_b)

    @property
    def joint_translation(self):
        """The current joint translation, usually in meters."""
        d = self.anchor_b - self.anchor_a
        return d.dot(self.axis)

    @property
    def joint_speed(self):
        """The current joint translation speed, usually in meters per second."""
        ba = self._body_a
        bb = self._body_b

        ca, cb = ba.world_center, bb.world_center
        qa = Mat22(angle=ba.angle)
        qb = Mat22(angle=bb.angle)

        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)
        p1 = ca + ra
        p2 = cb + rb
        d = p2 - p1
        axis = self._body_a.get_world_vector(self._local_x_axis)

        va = ba._linear_velocity
        vb = bb._linear_velocity
        wa = ba._angular_velocity
        wb = bb._angular_velocity

        speed = d.dot(scalar_cross(wa, axis)) + axis.dot(vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra))
        return speed

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
            self._impulse -= (0.0, 0.0, self._impulse.z)

    @property
    def lower_limit(self):
        """The lower translational limit (in meters)"""
        return self._lower_limit

    @lower_limit.setter
    def lower_limit(self, lower_limit):
        if self._lower_limit != lower_limit:
            for body in self.bodies:
                body.awake = True
            self._impulse -= (0.0, 0.0, self._impulse.z)
            self._lower_limit = lower_limit

    @property
    def upper_limit(self):
        """The upper translational limit (in meters)"""
        return self._upper_limit

    @upper_limit.setter
    def upper_limit(self, upper_limit):
        if self._upper_limit != upper_limit:
            for body in self.bodies:
                body.awake = True
            self._impulse -= (0.0, 0.0, self._impulse.z)
            self._upper_limit = upper_limit

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
    def motor_speed(self):
        """The current motor speed, usually in m/s."""
        return self._motor_speed

    @motor_speed.setter
    def motor_speed(self, speed):
        for body in self.bodies:
            body.awake = True
        self._motor_speed = speed

    @property
    def max_motor_force(self):
        """The maximum motor force, usually in N"""
        return self._max_motor_force

    @max_motor_force.setter
    def max_motor_force(self, force):
        for body in self.bodies:
            body.awake = True
        self._max_motor_force = force

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

        # Compute the effective masses.
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        d = cb + rb - ca - ra

        # Compute motor Jacobian and effective mass.
        axis = self._axis = qa * self._local_x_axis
        a1 = self._a1 = (d + ra).cross(axis)
        a2 = self._a2 = rb.cross(axis)

        self._motor_mass = ma + mb + ia * a1**2 + ib * a2**2
        if self._motor_mass > 0.0:
            self._motor_mass = 1.0 / self._motor_mass

        # Prismatic constraint.
        perp = self._perp = qa * self._local_y_axis

        s1 = self._s1 = (d + ra).cross(perp)
        s2 = self._s2 = rb.cross(perp)

        k11 = ma + mb + ia * (s1 ** 2) + ib * (s2 ** 2)
        k12 = ia * s1 + ib * s2
        k13 = ia * s1 * a1 + ib * s2 * a2
        k22 = ia + ib
        if k22 == 0.0:
            # For bodies with fixed rotation.
            k22 = 1.0
        k23 = ia * a1 + ib * a2
        k33 = ma + mb + ia * (a1 ** 2) + ib * (a2 ** 2)

        self._k = Mat33((k11, k12, k13), (k12, k22, k23), (k13, k23, k33))

        # Compute motor and limit terms.
        if self._limit_enabled:
            joint_translation = self._axis.dot(d)
            if abs(self._upper_limit - self._lower_limit) < (2.0 * LINEAR_SLOP):
                self._limit_state = EQUAL_LIMITS
            elif joint_translation <= self._lower_limit:
                if self._limit_state != AT_LOWER_LIMIT:
                    self._limit_state = AT_LOWER_LIMIT
                    self._impulse -= (0.0, 0.0, self._impulse.z)
            elif joint_translation >= self._upper_limit:
                if self._limit_state != AT_UPPER_LIMIT:
                    self._limit_state = AT_UPPER_LIMIT
                    self._impulse -= (0.0, 0.0, self._impulse.z)
            else:
                self._limit_state = INACTIVE_LIMIT
                self._impulse -= (0.0, 0.0, self._impulse.z)
        else:
            self._limit_state = INACTIVE_LIMIT
            self._impulse -= (0.0, 0.0, self._impulse.z)

        if self._motor_enabled == False:
            self._motor_impulse = 0.0

        if step.warm_starting:
            # Account for variable time step.
            self._impulse *= step.dt_ratio
            self._motor_impulse *= step.dt_ratio

            ix, iy, iz = self._impulse
            mi = self._motor_impulse

            P = ix * perp + (mi + iz) * axis
            L1 = ix * s1 + iy + (mi + iz) * a1
            L2 = ix * s2 + iy + (mi + iz) * a2

            va -= ma * P
            wa -= ia * L1

            vb += mb * P
            wb += ib * L2
        else:
            self._impulse = Vec3()
            self._motor_impulse = 0.0
        
        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        ra, rb = self._ra, self._rb

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib
        a1, a2 = self._a1, self._a2
        s1, s2 = self._s1, self._s2
        perp, axis = self._perp, self._axis

        # Solve linear motor constraint.
        if self._motor_enabled and self._limit_state != EQUAL_LIMITS:
            Cdot = self._axis.dot(vb - va) + a2 * wb - a1 * wa
            impulse = self._motor_mass * (self._motor_speed - Cdot)
            old_impulse = self._motor_impulse
            max_impulse = step.dt * self._max_motor_force
            self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
            impulse = self._motor_impulse - old_impulse

            P = impulse * self._axis
            L1 = impulse * a1
            L2 = impulse * a2

            va -= ma * P
            wa -= ia * L1

            vb += mb * P
            wb += ib * L2

        Cdot1 = Vec2(perp.dot(vb - va) + s2 * wb - s1 * wa, 
                     wb - wa)

        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT:
            # Solve prismatic and limit constraint in block form.
            Cdot2 = axis.dot(vb - va) + a2 * wb - a1 * wa
            Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

            old_impulse = Vec3(*self._impulse)
            df =  self._k.solve3x3(-Cdot)
            self._impulse += df

            if self._limit_state == AT_LOWER_LIMIT:
                iz = max(self._impulse.z, 0.0)
            elif self._limit_state == AT_UPPER_LIMIT:
                iz = min(self._impulse.z, 0.0)
            else:
                iz = self._impulse.z

            # f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - old_impulse(3))) + old_impulse(1:2)
            b = -Cdot1 - (self._impulse.z - old_impulse.z) * Vec2(self._k.col3.x, self._k.col3.y)
            f2r = self._k.solve2x2(b) + Vec2(old_impulse.x, old_impulse.y)

            self._impulse = Vec3(f2r.x, f2r.y, iz)
            df = self._impulse - old_impulse

            P = df.x * perp + df.z * axis
            L1 = df.x * s1 + df.y + df.z * a1
            L2 = df.x * s2 + df.y + df.z * a2

        else:
            # Limit is inactive, just solve the prismatic constraint in block form.
            df = self._k.solve2x2(-Cdot1)
            self._impulse += (df.x, df.y, 0.0)

            P = df.x * self._perp
            L1 = df.x * self._s1 + df.y
            L2 = df.x * self._s2 + df.y

        va -= ma * P
        wa -= ia * L1

        vb += mb * P
        wb += ib * L2

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_position_constraints(self, step, positions, velocities):
        """This returns True if the position errors are within tolerance."""
        index_a, index_b = self._indices
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib
        a1, a2 = self._a1, self._a2

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute fresh Jacobians
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        d = cb + rb - ca - ra

        axis = qa * self._local_x_axis
        a1 = (d + ra).cross(axis)
        a2 = rb.cross(axis)
        perp = qa * self._local_y_axis

        s1 = (d + ra).cross(perp)
        s2 = (rb).cross(perp)

        C1 = Vec2(perp.dot(d), ab - aa - self._reference_angle)

        # Solve linear limit constraint.
        linear_error, angular_error = abs(C1)
        active = False
        C2 = 0.0

        if self._limit_enabled:
            translation = axis.dot(d)
            if abs(self._upper_limit - self._lower_limit) < 2.0 * LINEAR_SLOP:
                # Prevent large angular corrections
                C2 = clamp(translation, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION)
                linear_error = max(linear_error, translation)
                active = True
            elif translation <= self._lower_limit:
                # Prevent large linear corrections and allow some slop.
                C2 = clamp(translation - self._lower_limit + LINEAR_SLOP, -MAX_LINEAR_CORRECTION, 0.0)
                linear_error = max(linear_error, self._lower_limit - translation)
                active = True
            elif translation >= self._upper_limit:
                # Prevent large linear corrections and allow some slop.
                C2 = clamp(translation - self._upper_limit - LINEAR_SLOP, 0.0, MAX_LINEAR_CORRECTION)
                linear_error = max(linear_error, translation - self._upper_limit)
                active = True

        if active:
            k11 = ma + mb + ia * (s1 ** 2) + ib * (s2 ** 2)
            k12 = ia * s1 + ib * s2
            k13 = ia * s1 * a1 + ib * s2 * a2
            k22 = ia + ib
            if k22 == 0.0:
                # For fixed rotation
                k22 = 1.0
            k23 = ia * a1 + ib * a2
            k33 = ma + mb + ia * (a1 ** 2) + ib * (a2 ** 2)

            self._k = Mat33((k11, k12, k13), (k12, k22, k23), (k13, k23, k33))
            C = Vec3(C1.x, C1.y, C2)

            impulse = self._k.solve3x3(-C)
        else:
            k11 = ma + mb + ia * s1 * s1 + ib * s2 * s2
            k12 = ia * s1 + ib * s2
            k22 = ia + ib
            if k22 == 0.0:
                k22 = 1.0

            self._k.col1 = Vec3(k11, k12, 0.0)
            self._k.col2 = Vec3(k12, k22, 0.0)

            impulse1 = self._k.solve2x2(-C1)
            impulse = (impulse1.x, impulse1.y, 0.0)

        ix, iy, iz = impulse
        P = ix * perp + iz * axis
        L1 = ix * s1 + iy + iz * a1
        L2 = ix * s2 + iy + iz * a2

        ca -= ma * P
        cb += mb * P
        aa -= ia * L1
        ab += ib * L2

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)
        return (linear_error <= LINEAR_SLOP) and (angular_error <= ANGULAR_SLOP)
        
