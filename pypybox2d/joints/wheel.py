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

__all__ = ('WheelJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (PI, Vec2, Mat22, scalar_cross, clamp, property)
from ..settings import LINEAR_SLOP
from .joint import Joint

class WheelJoint(Joint):
    """
    A wheel joint. This joint provides two degrees of freedom: translation
    along an axis fixed in body_a and rotation in the plane. You can use a
    joint limit to restrict the range of motion and a joint motor to drive
    the rotation or to model rotational friction.
    This joint is designed for vehicle suspensions.

    To create one requires defining a line of
    motion using an axis and an anchor point. The definition uses local
    anchor points and a local axis so that the initial configuration
    can violate the constraint slightly. The joint translation is zero
    when the local anchor points coincide in world space. Using local
    anchors and a local axis helps when saving and loading a game.
    """
    def __init__(self, body_a, body_b, anchor=None, axis=None, 
                 frequency=2.0, damping_ratio=0.7,
                 motor_enabled=False, motor_speed=0.0, max_motor_torque=0.0,
                 local_anchor_a=None, local_anchor_b=None,
                 local_x_axis=None, collide_connected=False):

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

        if local_x_axis is not None:
            self._local_x_axis = Vec2(*local_x_axis)
        else:
            self._local_x_axis = body_a.get_local_vector(axis)

        self._local_y_axis = scalar_cross(1.0, self._local_x_axis)
        self._frequency = float(frequency)
        self._damping_ratio = float(damping_ratio)
        self._max_motor_torque = float(max_motor_torque)
        self._motor_speed = float(motor_speed)
        self._motor_enabled = bool(motor_enabled)
        self._mass = 0.0
        self._impulse = 0.0
        self._motor_mass = 0.0
        self._motor_impulse = 0.0
        self._spring_mass = 0.0
        self._spring_impulse = 0.0
        self._bias = 0.0
        self._gamma = 0.0
        self._inv_i_a = 0.0
        self._inv_i_b = 0.0
        self._inv_mass_a = 0.0
        self._inv_mass_b = 0.0

        self._s_ax = 0.0
        self._s_bx = 0.0
        self._s_ay = 0.0
        self._s_by = 0.0
        self._ax = Vec2()
        self._ay = Vec2()

    def __copy__(self):
        return WheelJoint(self._body_a, self._body_b, None, None,
                          self._frequency, self._damping_ratio,
                          self._motor_enabled, self._motor_speed, self._max_motor_torque,
                          self._local_anchor_a, self._local_anchor_b,
                          self._local_x_axis, self._collide_connected)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * (self._impulse * self._ay + self._spring_impulse * self._ax)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._motor_impulse

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
    def joint_translation(self):
        """The current joint translation, usually in meters."""
        d = self.anchor_b - self.anchor_a
        axis = self._body_a.get_world_vector(self._local_x_axis)
        return d.dot(axis)

    @property
    def joint_speed(self):
        """The current joint translation speed, usually in meters per second."""
        return self._body_b._angular_velocity - self._body_a._angular_velocity

    @property
    def motor_enabled(self):
        """The joint motor enabled flag"""
        return self._motor_enabled

    @motor_enabled.setter
    def motor_enabled(self, motor_enabled):
        if motor_enabled != self._motor_enabled:
            for body in self.bodies:
                body.awake = True
            self._motor_enabled = motor_enabled

    @property
    def motor_speed(self):
        """The motor speed, usually in radians per second."""
        return self._motor_speed

    @motor_speed.setter
    def motor_speed(self, motor_speed):
        for body in self.bodies:
            body.awake = True
        self._motor_speed = motor_speed

    @property
    def max_motor_torque(self):
        """The maximum motor torque, usually in N-m."""
        return self._max_motor_torque

    @max_motor_torque.setter
    def max_motor_torque(self, max_motor_torque):
        for body in self.bodies:
            body.awake = True
        self._max_motor_torque = max_motor_torque

    def motor_torque(self, inv_dt):
        """Get the current motor torque given the inverse time step, usually in N-m."""
        return inv_dt * self._motor_impulse

    @property
    def spring_frequency(self):
        """The spring frequency in hertz. Setting the frequency to zero disables the spring."""
        return self._spring_frequency

    @spring_frequency.setter
    def spring_frequency(self, spring_frequency):
        self._spring_frequency = spring_frequency

    @property
    def spring_damping_ratio(self):
        """The spring damping ratio"""
        return self._spring_damping_ratio

    @spring_damping_ratio.setter
    def spring_damping_ratio(self, spring_damping_ratio):
        self._spring_damping_ratio = spring_damping_ratio

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

        # Compute the effective masses
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        d = cb + rb - ca - ra

        # Point to line constraint
        ay = self._ay = qa * self._local_y_axis
        s_ay = self._s_ay = (d + ra).cross(ay)
        s_by = self._s_by = rb.cross(ay)

        self._mass = ma + mb + ia * s_ay ** 2 + ib * s_by ** 2

        if self._mass > 0.0:
            self._mass = 1.0 / self._mass

        # Spring constraint
        self._spring_mass = 0.0
        self._bias = 0.0
        self._gamma = 0.0
        if self._frequency > 0.0:
            ax = self._ax = qa * self._local_x_axis
            s_ax = self._s_ax = (d + ra).cross(ax)
            s_bx = self._s_bx = rb.cross(ax)

            inv_mass = ma + mb + ia * (s_ax ** 2) + ib * (s_bx ** 2)

            if inv_mass > 0.0:
                self._spring_mass = 1.0 / inv_mass

                C = d.dot(ax)

                # Frequency
                omega = 2.0 * PI * self._frequency

                # Damping coefficient
                d = 2.0 * self._spring_mass * self._damping_ratio * omega

                # Spring stiffness
                k = self._spring_mass * (omega ** 2)

                # magic formulas
                dt = step.dt

                self._gamma = dt * (d + dt * k)
                if self._gamma > 0.0:
                    self._gamma = 1.0 / self._gamma

                self._bias = C * dt * k * self._gamma

                self._spring_mass = inv_mass + self._gamma
                if self._spring_mass > 0.0:
                    self._spring_mass = 1.0 / self._spring_mass
        else:
            self._spring_impulse = 0.0

        # Rotational motor
        if self._motor_enabled:
            self._motor_mass = ia + ib
            if self._motor_mass > 0.0:
                self._motor_mass = 1.0 / self._motor_mass
        else:
            self._motor_mass = 0.0
            self._motor_impulse = 0.0

        if step.warm_starting:
            # Account for variable time step.
            dt_ratio = step.dt_ratio

            self._impulse *= dt_ratio
            self._spring_impulse *= dt_ratio
            self._motor_impulse *= dt_ratio

            impulse = self._impulse
            spring_impulse = self._spring_impulse
            motor_impulse = self._motor_impulse

            P = impulse * ay + spring_impulse * self._ax
            La = impulse * s_ay + spring_impulse * self._s_ax + motor_impulse
            Lb = impulse * s_by + spring_impulse * self._s_bx + motor_impulse

            va -= ma * P
            wa -= ia * La

            vb += mb * P
            wb += ib * Lb
        else:
            self._impulse = 0.0
            self._spring_impulse = 0.0
            self._motor_impulse = 0.0

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        ra, rb = self._ra, self._rb
        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Solve spring constraint
        Cdot = self._ax.dot(vb - va) + self._s_bx * wb - self._s_ax * wa
        impulse = -self._spring_mass * (Cdot + self._bias + self._gamma * self._spring_impulse)
        self._spring_impulse += impulse

        P = impulse * self._ax
        La = impulse * self._s_ax
        Lb = impulse * self._s_bx

        va -= ma * P
        wa -= ia * La

        vb += mb * P
        wb += ib * Lb

        # Solve rotational motor constraint
        Cdot = wb - wa - self._motor_speed
        impulse = -self._motor_mass * Cdot

        old_impulse = self._motor_impulse
        max_impulse = step.dt * self._max_motor_torque
        self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
        impulse = self._motor_impulse - old_impulse

        wa -= ia * impulse
        wb += ib * impulse

        # Solve point to line constraint
        Cdot = self._ay.dot(vb - va) + self._s_by * wb - self._s_ay * wa
        impulse = -self._mass * Cdot
        self._impulse += impulse

        P = impulse * self._ay
        La = impulse * self._s_ay
        Lb = impulse * self._s_by

        va -= ma * P
        wa -= ia * La

        vb += mb * P
        wb += ib * Lb

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

        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        d = (cb - ca) + rb - ra

        k = ma + mb + ia * self._s_ay ** 2 + ib * self._s_by ** 2

        ay = qa * self._local_y_axis
        s_ay = (d + ra).cross(ay)
        s_by = rb.cross(ay)

        C = d.dot(ay)

        if k != 0.0:
            impulse = - C / k
        else:
            impulse = 0.0

        P = impulse * ay
        La = impulse * s_ay
        Lb = impulse * s_by

        ca = ca - ma * P
        aa -= ia * La
        cb = cb + mb * P
        ab += ib * Lb

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)
        return abs(C) <= LINEAR_SLOP

