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

__all__ = ('Joint', 'DistanceJoint', 'RevoluteJoint', 'FrictionJoint', 
           'PrismaticJoint', 'WeldJoint', 'RopeJoint', 'WheelJoint', 
           'MouseJoint', 'PulleyJoint', 'GearJoint',
           'INACTIVE_LIMIT', 'AT_LOWER_LIMIT', 'AT_UPPER_LIMIT', 'EQUAL_LIMITS', 
           'ALLOWED_STRETCH'
          )

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import (PI, Vec2, Vec3, Mat22, Mat33, 
                     scalar_cross, clamp, is_valid_float, property)
from .settings import (EPSILON, LINEAR_SLOP, ANGULAR_SLOP, MAX_LINEAR_CORRECTION, MAX_ANGULAR_CORRECTION)

# TODO: __init__ doc strings

INACTIVE_LIMIT, AT_LOWER_LIMIT, AT_UPPER_LIMIT, EQUAL_LIMITS = range(4)
ALLOWED_STRETCH = 10.0 * LINEAR_SLOP

class Joint(object):
    """
    The base joint class. Joints are used to constraint two bodies together in
    various fashions. Some joints also feature limits and motors.
    """

    def __init__(self, body_a, body_b, collide_connected):
        if body_a and body_b:
            if body_a._world != body_b.world:
                raise ValueError('Both bodies must be in the same world.')
            if body_a._world is None or body_b._world is None:
                raise ValueError('Both bodies must be in the same world.')

        self._body_a = body_a
        self._body_b = body_b
        self._island_flag = False
        self._collide_connected = collide_connected
        self._inv_mass_a = 0.0
        self._inv_mass_b = 0.0
        self._inv_Ia = 0.0
        self._inv_Ib = 0.0

    @property
    def body_a(self):
        """Get the first body attached to this joint."""
        return self._body_a

    @property
    def body_b(self):
        """Get the second body attached to this joint."""
        return self._body_b

    @property
    def bodies(self):
        """Get both bodies attached to this joint"""
        return [self._body_a, self._body_b]

    def other_body(self, body):
        """Given body_a, return body_b or vice-versa."""
        if body == self._body_a:
            return self._body_b
        else:
            assert(body == self._body_b) # TODO remove
            return self._body_a


    @property
    def active(self):
        """Short-cut function to determine if either body is inactive."""
        return self._body_a.active and self._body_b.active

    @property
    def collide_connected(self):
        """
        Get collide connected.
        Note: modifying the collide connect flag won't work correctly because
        the flag is only checked when fixture AABBs begin to overlap.
        """
        return self._collide_connected

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * Vec2(self._impulse.x, self._impulse.y)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._impulse.z

    def _init_velocity_constraints(self, step, positions, velocities):
        raise NotImplementedError # Implemented in subclass
    def _solve_velocity_constraints(self, step, positions, velocities):
        raise NotImplementedError # Implemented in subclass
    def _solve_position_constraints(self, step, positions, velocities):
        """This returns True if the position errors are within tolerance."""
        raise NotImplementedError # Implemented in subclass


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

        length = u.normalize()
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
            self._reference_angle = reference_angle
        else:
            self._reference_angle = body_b.angle - body_a.angle

        self._impulse = Vec3()
        self._lower_angle = lower_angle
        self._upper_angle = upper_angle
        self._max_motor_torque = max_motor_torque
        self._limit_enabled = limit_enabled
        self._motor_enabled = motor_enabled
        self._motor_impulse = 0.0
        self._motor_speed = motor_speed
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
            self._impulse.z = 0.0

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
            self._impulse.z = 0.0
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
            self._impulse.z = 0.0
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

        # TODO: when rewriting Mat33 in C, these will need to be _col1, etc.
        self._mass.col1.x = ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib
        self._mass.col2.x = -ra.y * ra.x * ia - rb.y * rb.x * ib
        self._mass.col3.x = -ra.y * ia - rb.y * ib
        self._mass.col1.y = self._mass.col2.x
        self._mass.col2.y = ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib
        self._mass.col3.y = ra.x * ia + rb.x * ib
        self._mass.col1.z = self._mass.col3.x
        self._mass.col2.z = self._mass.col3.y
        self._mass.col3.z = ia + ib

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
                    self._impulse.z = 0.0
                self._limit_state = AT_LOWER_LIMIT
            elif joint_angle >= self._upper_angle:
                if self._limit_state != AT_UPPER_LIMIT:
                    self._impulse.z = 0.0
                self._limit_state = AT_UPPER_LIMIT
            else:
                self._limit_state = INACTIVE_LIMIT
                self._impulse.z = 0.0
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
            self._impulse.zero()
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
                    impulse.x = reduced.x
                    impulse.y = reduced.y
                    impulse.z = -self._impulse.z
                    self._impulse.x += reduced.x
                    self._impulse.y += reduced.y
                    self._impulse.z = 0.0
                else:
                    self._impulse += impulse
            elif self._limit_state == AT_UPPER_LIMIT:
                new_impulse = self._impulse.z + impulse.z
                if new_impulse > 0.0:
                    rhs = -Cdot1 + self._impulse.z * Vec2(self._mass.col3.x, self._mass.col3.y)
                    reduced = self._mass.solve2x2(rhs)
                    impulse.x = reduced.x
                    impulse.y = reduced.y
                    impulse.z = -self._impulse.z
                    self._impulse.x += reduced.x
                    self._impulse.y += reduced.y
                    self._impulse.z = 0.0
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

            self._impulse.x += impulse.x
            self._impulse.y += impulse.y

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

            aa -= self._inv_Ia * limit_impulse
            ab += self._inv_Ib * limit_impulse

        # Solve point-to-point constraint.
        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        C = cb + rb - ca - ra
        position_error = C.length

        ma, mb = self._inv_mass_a, self._inv_mass_b
        inv_ia, inv_ib = self._inv_Ia, self._inv_Ib

        K = Mat22()
        K.col1.x = ma + mb + ia * ra.y * ra.y + ib * rb.y * rb.y
        K.col1.y = -ia * ra.x * ra.y - ib * rb.x * rb.y
        K.col2.x = K.col1.y
        K.col2.y = ma + mb + ia * ra.x * ra.x + ib * rb.x * rb.x

        impulse = -K.solve(C)

        ca -= ma * impulse
        aa -= inv_ia * ra.cross(impulse)

        cb += mb * impulse
        ab += inv_ib * rb.cross(impulse)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

        return position_error <= LINEAR_SLOP and angular_error <= ANGULAR_SLOP

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
        K.col1.x = inv_mass_a + inv_mass_b + inv_Ia * ra.y * ra.y + inv_Ib * rb.y * rb.y
        K.col1.y = -inv_Ia * ra.x * ra.y - inv_Ib * rb.x * rb.y
        K.col2.x = K.col1.y
        K.col2.y = inv_mass_a + inv_mass_b + inv_Ia * ra.x * ra.x + inv_Ib * rb.x * rb.x

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
            self._linear_impulse.zero()
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
            self._linear_impulse.normalize()
            self._linear_impulse *= max_impulse

        impulse = self._linear_impulse - old_impulse

        va -= inv_mass_a * impulse
        wa -= inv_Ia * ra.cross(impulse)

        vb += inv_mass_b * impulse
        wb += inv_Ib * rb.cross(impulse)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

    def _solve_position_constraints(self, step, positions, velocities):
        return True

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
        return self._body_a.get_world_vector(self._local_x_axis);

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

        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)
        p1 = ca + ra
        p2 = cb + rb
        d = p2 - p1
        axis = self._body_a.get_world_vector(self._local_x_axis);

        va = va
        vb = vb
        wa = aa
        wb = ab

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
            self._impulse.z = 0.0

    @property
    def lower_limit(self):
        """The lower translational limit (in meters)"""
        return self._lower_limit

    @lower_limit.setter
    def lower_limit(self, lower_limit):
        if self._lower_limit != lower_limit:
            for body in self.bodies:
                body.awake = True
            self._impulse.z = 0.0
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
            self._impulse.z = 0.0
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
                    self._impulse.z = 0.0
            elif joint_translation >= self._upper_limit:
                if self._limit_state != AT_UPPER_LIMIT:
                    self._limit_state = AT_UPPER_LIMIT
                    self._impulse.z = 0.0
            else:
                self._limit_state = INACTIVE_LIMIT
                self._impulse.z = 0.0
        else:
            self._limit_state = INACTIVE_LIMIT
            self._impulse.z = 0.0

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
            self._impulse.zero()
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
                self._impulse.z = max(self._impulse.z, 0.0)
            elif self._limit_state == AT_UPPER_LIMIT:
                self._impulse.z = min(self._impulse.z, 0.0)

            # f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - old_impulse(3))) + old_impulse(1:2)
            b = -Cdot1 - (self._impulse.z - old_impulse.z) * Vec2(self._k.col3.x, self._k.col3.y)
            f2r = self._k.solve2x2(b) + Vec2(old_impulse.x, old_impulse.y)
            self._impulse.x = f2r.x
            self._impulse.y = f2r.y

            df = self._impulse - old_impulse

            P = df.x * perp + df.z * axis
            L1 = df.x * s1 + df.y + df.z * a1
            L2 = df.x * s2 + df.y + df.z * a2

        else:
            # Limit is inactive, just solve the prismatic constraint in block form.
            df = self._k.solve2x2(-Cdot1)
            self._impulse.x += df.x
            self._impulse.y += df.y

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
        ba = self._body_a
        bb = self._body_b

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
        
class WeldJoint(Joint):
    """
    A weld joint essentially glues two bodies together. A weld joint may
    distort somewhat because the island constraint solver is approximate.
    
    In creating one, you need to specify either one world anchor point or
    two local anchor points where they are attached and the relative body
    angle. The position of the anchor points is important for computing 
    the reaction torque.
    """
    # Point-to-point constraint
    # C = p2 - p1
    # Cdot = vb - va
    #      = vb + cross(wb, rb) - va - cross(wa, ra)
    # J = [-I -ra_skew I rb_skew ]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)

    # Angle constraint
    # C = angle2 - angle1 - referenceAngle
    # Cdot = wb - wa
    # J = [0 0 -1 0 0 1]
    # K = invIa + invIb
    def __init__(self, body_a, body_b, anchor=None, reference_angle=None,
                 local_anchor_a=None, local_anchor_b=None,
                 collide_connected=False):
        """

        """
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        if anchor is not None:
            self._local_anchor_a = body_a.get_local_point(anchor)
            self._local_anchor_b = body_b.get_local_point(anchor)
        else:
            self._local_anchor_a = Vec2(*local_anchor_a)
            self._local_anchor_b = Vec2(*local_anchor_b)
  
        if reference_angle is not None:
            self._reference_angle = float(reference_angle)
        else:
            self._reference_angle = body_b.angle - body_a.angle

        self._mass = Mat33()
        self._impulse = Vec3()

    def __copy__(self):
        return WeldJoint(self._body_a, self._body_b, None, self._reference_angle,
                         self._local_anchor_a, self._local_anchor_b,
                         self._collide_connected)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * Vec2(self._impulse.x, self._impulse.y)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._impulse.z

    @property
    def anchor_a(self):
        """Get the anchor point on body_a in world coordinates"""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """Get the anchor point on body_b in world coordinates"""
        return self._body_b.get_world_point(self._local_anchor_b)

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

        c2x = -ra.y * ra.x * ia - rb.y * rb.x * ib
        c3x = -ra.y * ia - rb.y * ib
        self._mass.col1.x = ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib
        self._mass.col2.x = c2x
        self._mass.col3.x = c3x
        self._mass.col1.y = c2x
        self._mass.col2.y = ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib
        self._mass.col3.y = ra.x * ia + rb.x * ib
        self._mass.col1.z = c3x
        self._mass.col2.z = c3x
        self._mass.col3.z = ia + ib

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._impulse *= step.dt_ratio

            ix, iy, iz = self._impulse
            P = Vec2(ix, iy)

            va -= ma * P
            wa -= ia * (ra.cross(P) + iz)

            vb += mb * P
            wb += ib * (rb.cross(P) + iz)
        else:
            self._impulse.zero()
        
        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        ra, rb = self._ra, self._rb
        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Solve point-to-point constraint
        Cdot1 = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)
        Cdot2 = wb - wa
        Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

        impulse = self._mass.solve3x3(-Cdot)
        self._impulse += impulse

        P = Vec2(impulse.x, impulse.y)

        va -= ma * P
        wa -= ia * (ra.cross(P) + impulse.z)

        vb += mb * P
        wb += ib * (rb.cross(P) + impulse.z)

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

        C1 = cb + rb - ca - ra
        C2 = ab - aa - self._reference_angle

        position_error = C1.length
        angular_error = abs(C2)

        mass = self._mass
        mass.col1.x = ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib
        mass.col2.x = c2x = -ra.y * ra.x * ia - rb.y * rb.x * ib
        mass.col3.x = c3x = -ra.y * ia - rb.y * ib
        mass.col1.y = c2x
        mass.col2.y = ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib
        mass.col3.y = c3y = ra.x * ia + rb.x * ib
        mass.col1.z = c3x
        mass.col2.z = c3y
        mass.col3.z = ia + ib

        C = Vec3(C1.x, C1.y, C2)
        impulse = -mass.solve3x3(C)

        ix, iy, iz = impulse
        P = Vec2(ix, iy)
        ca -= ma * P
        aa -= ia * (ra.cross(P) + iz)

        cb += mb * P
        ab += ib * (rb.cross(P) + iz)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

        return position_error <= LINEAR_SLOP and angular_error <= ANGULAR_SLOP

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
            u *= 1.0 / self._length
        else:
            u.zero()
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

        length = u.normalize()
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
        axis = self._body_a.get_world_vector(self._local_x_axis);
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

        # Compute the effective mass matrix.
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        d = cb + rb - ca - ra

        # Point to line constraint
        ay = self._ay = qa * (self._local_y_axis)
        s_ay = self._s_ay = (d + ra).cross(self._ay)
        s_by = self._s_by = rb.cross(self._ay)

        self._mass = ma + mb + ia * s_ay ** 2 + ib * s_by ** 2

        if self._mass > 0.0:
            self._mass = 1.0 / self._mass

        # Spring constraint
        self._spring_mass = 0.0
        self._bias = 0.0
        self._gamma = 0.0
        if self._frequency > 0.0:
            ax = self._ax = qa * self._local_x_axis
            s_ax = self._s_ax = (d + ra).cross(self._ax)
            s_bx = self._s_bx = rb.cross(self._ax)

            inv_mass = ma + mb + ia * s_ax ** 2 + ib * s_bx ** 2

            if inv_mass > 0.0:
                self._spring_mass = 1.0 / inv_mass

                C = d.dot(ax)

                # Frequency
                omega = 2.0 * PI * self._frequency

                # Damping coefficient
                d = 2.0 * self._spring_mass * self._damping_ratio * omega

                # Spring stiffness
                k = self._spring_mass * omega * omega

                dt = step.dt

                # magic formulas
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

            P = self._impulse * ay + self._spring_impulse * self._ax
            La = self._impulse * s_ay + self._spring_impulse * self._s_ax + self._motor_impulse
            Lb = self._impulse * s_by + self._spring_impulse * self._s_bx + self._motor_impulse

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
        impulse = self._mass * (-Cdot)
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

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        d = cb + rb - ca - ra

        ay = qa * self._local_y_axis
        s_ay = (d + ra).cross(ay)
        s_by = rb.cross(ay)

        C = d.dot(ay)

        k = ma + mb + ia * self._s_ay ** 2 + ib * self._s_by ** 2 # TODO: wrong?

        if k != 0.0:
            impulse = - C / k
        else:
            impulse = 0.0

        P = impulse * ay
        La = impulse * s_ay
        Lb = impulse * s_by

        ca -= ma * P
        aa -= ia * La
        cb += mb * P
        ab += ib * Lb

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)
        return abs(C) <= LINEAR_SLOP


class MouseJoint(Joint):
    """
    A mouse joint is used to make a point on a body track a
    specified world point. This a soft constraint with a maximum
    force. This allows the constraint to stretch and without
    applying huge forces.

    Creation requires a world target point, tuning parameters, and 
    the time step.
    
    NOTE: this joint is not documented in the manual because it was
    developed to be used in the testbed. If you want to learn how to
    use the mouse joint, look at the testbed.
    """
    # p = attached point, m = mouse point
    # C = p - m
    # Cdot = v
    #      = v + cross(w, r)
    # J = [I r_skew]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)
    def __init__(self, body, target=(0, 0), max_force = 0.0, frequency=5.0, damping_ratio=0.7):
        if body is None:
            raise ValueError('body must be set')

        target = Vec2(*target)

        if not target.valid:
            raise ValueError('Invalid target')
        if not is_valid_float(max_force) or max_force < 0.0:
            raise ValueError('Invalid maximum force')
        if not is_valid_float(frequency) or frequency < 0.0:
            raise ValueError('Invalid frequency')
        if not is_valid_float(damping_ratio) or damping_ratio < 0.0:
            raise ValueError('Invalid damping ratio')

        Joint.__init__(self, None, body, False)

        self._target = target
        self._local_anchor_b = body.get_local_point(target)
        self._max_force = max_force
        self._impulse = Vec2()
        self._frequency = frequency
        self._damping_ratio = damping_ratio
        
        self._beta = 0.0
        self._gamma = 0.0

    def __copy__(self):
        return MouseJoint(self._body_b, self._target,
                          self._max_force, self._frequency, self._damping_ratio)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * self._impulse

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return 0.0 # inv_dt * 0.0

    @property
    def target(self):
        """
        The target point.  This is assumed to coincide with the body
        anchor initially.
        """
        return Vec2(*self._target)

    @target.setter
    def target(self, target):
        if not self._body_b.awake:
            self._body_b.awake = True

        self._target = Vec2(*target)

    @property
    def max_force(self):
        """
        The maximum constraint force that can be exerted
        to move the candidate body. Usually you will express
        as some multiple of the weight (multiplier * mass * gravity).
        """
        return self._max_force

    @max_force.setter
    def max_force(self, max_force):
        self._max_force = max_force

    @property
    def frequency(self):
        """The response speed"""
        return self._frequency

    @frequency.setter
    def frequency(self, frequency):
        self._frequency = frequency

    @property
    def damping_ratio(self):
        """The damping ratio: 0 = no damping, 1 = critical damping"""
        return self._damping_ratio

    @damping_ratio.setter
    def damping_ratio(self, damping_ratio):
        self._damping_ratio = damping_ratio

    def _init_velocity_constraints(self, step, positions, velocities):
        body = self._body_b
        self._index = index_b = body._island_index

        cb, ab = positions[index_b]
        vb, wb = velocities[index_b]
        mb = self._inv_mass_b = body._inv_mass
        ib = self._inv_Ib = body._invI
        self._local_center_b = body._sweep.local_center
        qb = Mat22(angle=ab)

        self._mass = mass = body.mass

        # Frequency
        omega = 2.0 * PI * self._frequency

        # Damping coefficient
        d = 2.0 * mass * self._damping_ratio * omega

        # Spring stiffness
        k = mass * (omega * omega)

        # magic formulas
        # gamma has units of inverse mass.
        # beta has units of inverse time.
        dt = step.dt

        assert(d + dt * k > EPSILON)
        self._gamma = dt * (d + dt * k)
        if self._gamma != 0.0:
            self._gamma = 1.0 / self._gamma
        self._beta = dt * k * self._gamma

        # Compute the effective mass matrix.
        rb = self._rb = qb * (self._local_anchor_b - self._local_center_b)

        # K    = [(1/ma + 1/mb) * eye(2) - skew(ra) * invIa * skew(ra) - skew(rb) * invIb * skew(rb)]
        #      = [1/ma+1/mb     0    ] + invIa * [ra.y*ra.y -ra.x*ra.y] + invIb * [ra.y*ra.y -ra.x*ra.y]
        #        [    0     1/ma+1/mb]           [-ra.x*ra.y ra.x*ra.x]           [-ra.x*ra.y ra.x*ra.x]

        K = Mat22()
        K.col1.x = mb + ib * rb.y ** 2 + self._gamma
        K.col1.y = -ib * rb.x * rb.y
        K.col2.x = K.col1.y
        K.col2.y = mb + ib * rb.x ** 2 + self._gamma

        self._mass = K.inverse

        self._c = self._beta * (cb + rb - self._target)

        # Cheat with some damping
        wb *= 0.98

        if step.warm_starting:
            # Warm starting.
            self._impulse *= step.dt_ratio
            vb += mb * self._impulse
            wb += ib * rb.cross(self._impulse)
        else:
            self._impulse.zero()

        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_b = self._index

        cb, ab = positions[index_b]
        vb, wb = velocities[index_b]
        mb = self._inv_mass_b
        ib = self._inv_Ib
        rb = self._rb

        # Cdot = v + cross(w, r)
        Cdot = vb + scalar_cross(wb, rb)
        impulse = self._mass * (-(Cdot + self._c + self._gamma * self._impulse))

        old_impulse = Vec2(*self._impulse)
        self._impulse += impulse
        max_impulse = step.dt * self._max_force
        if self._impulse.length_squared > max_impulse ** 2:
            self._impulse *= max_impulse / self._impulse.length
        impulse = self._impulse - old_impulse

        vb += mb * impulse
        wb += ib * rb.cross(impulse)
    
        velocities[index_b] = (vb, wb)

    def _solve_position_constraints(self, step, positions, velocities):
        """This returns true if the position errors are within tolerance."""
        return True

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
        ua = self._ua = ca + ra - self._ground_anchor_a
        ub = self._ub = cb + rb - self._ground_anchor_b

        length_a = ua.length
        length_b = ub.length

        if length_a > 10.0 * LINEAR_SLOP:
            self._ua *= 1.0 / length_a
        else:
            self._ua.zero()

        if length_b > 10.0 * LINEAR_SLOP:
            self._ub *= 1.0 / length_b
        else:
            self._ub.zero()

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
            Pa = -(self._impulse) * ua
            Pb = (-self._ratio * self._impulse) * ub
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
            ua *= 1.0 / length_a
        else:
            ua.zero()

        if length_b > 10.0 * LINEAR_SLOP:
            ub *= 1.0 / length_b
        else:
            ub.zero()

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


class GearJoint(Joint):
    """
    A gear joint is used to connect two joints together. Either joint
    can be a revolute or prismatic joint. You specify a gear ratio
    to bind the motions together:
    coordinate1 + ratio * coordinate2 = constant
    The ratio can be negative or positive. If one joint is a revolute joint
    and the other joint is a prismatic joint, then the ratio will have units
    of length or units of 1/length.
    @warning The revolute and prismatic joints must be attached to
    fixed bodies (which _must be_ body_a on those joints).

    Creation requires two existing
    revolute or prismatic joints (any combination will work).
    The provided joints must attach a dynamic body to a static body.
    """
    # Gear Joint:
    # C0 = (coordinate1 + ratio * coordinate2)_initial
    # C = C0 - (cordinate1 + ratio * coordinate2) = 0
    # Cdot = -(Cdot1 + ratio * Cdot2)
    # J = -[J1 ratio * J2]
    # K = J * invM * JT
    #   = J1 * invMa * J1T + ratio * ratio * J2 * invMb * J2T
    #
    # Revolute:
    # coordinate = rotation
    # Cdot = angularVelocity
    # J = [0 0 1]
    # K = J * invM * JT = invI
    #
    # Prismatic:
    # coordinate = dot(p - pg, ug)
    # Cdot = dot(v + cross(w, r), ug)
    # J = [ug cross(r, ug)]
    # K = J * invM * JT = invMass + invI * cross(r, ug)^2
    def __init__(self, joint_a, joint_b, ratio=0.1):
        Joint.__init__(self, None, None, False)

        if not isinstance(joint_a, (RevoluteJoint, PrismaticJoint)):
            raise ValueError('Joints must be either Revolute or Prismatic')
        if not isinstance(joint_b, (RevoluteJoint, PrismaticJoint)):
            raise ValueError('Joints must be either Revolute or Prismatic')

        self._joint_a = joint_a
        self._joint_b = joint_b

        # Get geometry of joint_a
        body_a = self._body_a = joint_a.body_b
        body_c = self._body_c = joint_a.body_a

        xf_a, xf_c = body_a._xf, body_c._xf
        aa, ac = body_a._sweep.a, body_c._sweep.a

        self._revolute_a = isinstance(joint_a, RevoluteJoint)
        if self._revolute_a:
            revolute = joint_a
            self._local_anchor_c = Vec2(*revolute._local_anchor_a)
            self._local_anchor_a = Vec2(*revolute._local_anchor_b)
            self._reference_angle_a = revolute._reference_angle
            self._local_axis_c = Vec2()

            coordinate_a = aa - ac - self._reference_angle_a
        else:
            prismatic = joint_a
            self._local_anchor_c = Vec2(*prismatic._local_anchor_a)
            self._local_anchor_a = Vec2(*prismatic._local_anchor_b)
            self._reference_angle_a = prismatic._reference_angle
            self._local_axis_c = Vec2(*prismatic._local_x_axis)
            
            pc = self._local_anchor_c
            pa = xf_c.rotation.mul_t(xf_a.rotation * self._local_anchor_a + (xf_a.position - xf_c.position))
            coordinate_a = (pa - pc).dot(self._local_axis_c)

        # Get geometry of joint_b
        body_d = self._body_d = joint_a.body_a
        body_b = self._body_b = joint_a.body_b

        xf_b, xf_d = body_b._xf, body_d._xf
        ab, ad = body_b._sweep.a, body_d._sweep.a
        self._revolute_b = isinstance(joint_b, RevoluteJoint)
        if self._revolute_b:
            revolute = joint_b
            self._local_anchor_d = Vec2(*revolute._local_anchor_a)
            self._local_anchor_b = Vec2(*revolute._local_anchor_b)
            self._reference_angle_b = revolute._reference_angle
            self._local_axis_d = Vec2()

            coordinate_b = ab - ad - self._reference_angle_b
        else:
            prismatic = joint_b
            self._local_anchor_d = Vec2(*prismatic._local_anchor_a)
            self._local_anchor_b = Vec2(*prismatic._local_anchor_b)
            self._reference_angle_b = prismatic._reference_angle
            self._local_axis_d = Vec2(*prismatic._local_x_axis)
            
            pd = self._local_anchor_d
            pb = xf_d.rotation.mul_t(xf_b.rotation * self._local_anchor_b + (xf_b.position - xf_d.position))
            coordinate_b = (pb - pd).dot(self._local_axis_d)

        self._ground_anchor_b = Vec2(*joint_b._local_anchor_a)
        self._local_anchor_b = Vec2(*joint_b._local_anchor_b)
        
        self._ratio = ratio
        self._constant = coordinate_a + self._ratio * coordinate_b
        self._impulse = 0.0
        self._bodies = (self._body_a, self._body_b, self._body_c, self._body_d)

    def __copy__(self):
        return GearJoint(self._joint_a, self._joint_b, self._ratio)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * (self._impulse * self._j.linear_b) # TODO_ERIN not tested

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        r = self._body_b._xf._rotation * (self._local_anchor_b - self._body_b._sweep.local_center)
        P = self._impulse * self._j.linear_b
        L = self._impulse * self._j.angular_b - r.cross(P)
        return inv_dt * L # TODO_ERIN not tested

    @property
    def joint_a(self):
        return self._joint_a

    @property
    def joint_b(self):
        return self._joint_b

    @property
    def revolute_joint_a(self):
        return self._revolute_a

    @property
    def revolute_joint_b(self):
        return self._revolute_b

    @property
    def prismatic_joint_a(self):
        return self._prismatic_a

    @property
    def prismatic_joint_b(self):
        return self._prismatic_b

    @property
    def ratio(self):
        return self._ratio

    @ratio.setter
    def ratio(self, ratio):
        if not is_valid_float(ratio):
            raise ValueError('Invalid ratio')
        self._ratio = ratio

    @property
    def anchor_a(self):
        """The world anchor point relative to body_a's origin."""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """The world anchor point relative to body_b's origin."""
        return self._body_b.get_world_point(self._local_anchor_b)

    def _init_velocity_constraints(self, step, positions, velocities):
        bodies = self._bodies
        self._indices = [body._island_index for body in bodies]
        self._local_centers = [body._sweep.local_center for body in bodies]
        self._masses = [body._inv_mass for body in bodies]
        self._inertias = [body._invI for body in bodies]

        index_a, index_b, index_c, index_d = indices = self._indices
        Cs = self._centers = [positions[index][0] for index in indices]
        As = self._angles = [positions[index][1] for index in indices]

        Vs = self._lin_vels = [velocities[index][0] for index in indices]
        Ws = self._ang_vels = [velocities[index][1] for index in indices]
      
        Qs = [Mat22(angle=a) for a in As]

        ca, cb, cc, cd = Cs
        aa, ab, ac, ad = As
        ia, ib, ic, id_ = self._inertias
        ma, mb, mc, md = self._masses
        qa, qb, qc, qd = Qs
        lca, lcb, lcc, lcd = self._local_centers

        self._mass = 0.0

        if self._revolute_a:
            self._j_vac = Vec2()
            self._j_wa = 1.0
            self._j_wc = 1.0
            self._mass += ia + ic
        else:
            u = qc * self._local_axis_c
            rc = qc * (self._local_anchor_c - lcc)
            ra = qa * (self._local_anchor_a - lca)
            self._j_vac = u
            self._j_wc = rc.cross(u)
            self._j_wa = ra.cross(u)
            self._mass += mc + ma + ic * self._j_wc * self._j_wc + ia * self._j_wa * self._j_wa

        if self._revolute_b:
            self._j_vbd = Vec2()
            self._j_wb = 1.0
            self._j_wd = 1.0
            self._mass += self._ratio**2 * (ib + id_)
        else:
            u = qd * self._local_axis_d
            rd = qd * (self._local_anchor_d - lcd)
            rb = qb * (self._local_anchor_b - lcb)
            self._j_vbd = u
            self._j_wd = rd.cross(u)
            self._j_wb = rb.cross(u)
            self._mass += self._ratio**2 * (md + mb) + id_ * self._j_wd ** 2 + ib * self._j_wb * self._j_wb

        # Compute effective mass.
        if self._mass > 0.0:
            self._mass = 1.0 / self._mass
        else:
            self._mass = 0.0

        # Linear velocity Jacobian values corresponding to lin vel a~d
        self._lv_j_values = lv_j_values = [self._j_vac, self._j_vbd] * 2

        # Angular velocity Jacobian values corresponding to ang vel a~d
        self._av_j_values = av_j_values = [self._j_wa, self._j_wb, self._j_wc, self._j_wd]

        if step.warm_starting:
            # Warm starting.
            impulse = self._impulse
            for k, (i, m, lvj, avj) in enumerate(zip(self._inertias, self._masses, lv_j_values, av_j_values)):
                Vs[k] += (m * impulse) * lvj
                Ws[k] += (i * impulse) * avj

            #Vs[0] += (ma * impulse) * self._j_vac
            #Ws[0] += ia * impulse * self._j_wa

            #Vs[1] += (mb * impulse) * self._j_vbd
            #Ws[1] += ib * impulse * self._j_wb

            #Vs[2] -= (mc * impulse) * self._j_vac
            #Ws[2] -= ic * impulse * self._j_wc

            #Vs[3] -= (md * impulse) * self._j_vbd
            #Ws[3] -= id_ * impulse * self._j_wd
        else:
            self._impulse = 0.0

        for index, v, w in zip(self._indices, Vs, Ws):
            velocities[index] = (v, w)

    def _solve_velocity_constraints(self, step, positions, velocities):
        indices = self._indices
        Vs = self._lin_vels
        Ws = self._ang_vels = [velocities[index][1] for index in indices]

        va, vb, vc, vd = Vs
        wa, wb, wc, wd = Ws

        Cdot = self._j_vac.dot(va - vc) + self._j_vbd.dot(vb - vd)
        Cdot += (self._j_wa * wa - self._j_wc * wc) + (self._j_wb * wb - self._j_wd * wd)

        impulse = -self._mass * Cdot
        self._impulse += impulse

        for k, (i, m, lvj, avj) in enumerate(zip(self._inertias, self._masses, self._lv_j_values, self._av_j_values)):
            Vs[k] += (m * impulse) * lvj
            Ws[k] += (i * impulse) * avj

        for index, v, w in zip(self._indices, Vs, Ws):
            velocities[index] = (v, w)

    def _solve_position_constraints(self, step, positions, velocities):
        indices = self._indices
        ratio = self._ratio
        Cs = self._centers
        As = self._angles

        Qs = [Mat22(angle=a) for a in As]
        qa, qb, qc, qd = Qs
        ca, cb, cc, cd = Cs
        aa, ab, ac, ad = As

        ia, ib, ic, id_ = self._inertias
        ma, mb, mc, md = self._masses
        lca, lcb, lcc, lcd = self._local_centers

        mass = 0.0
        if self._revolute_a:
            j_vac = Vec2()
            j_wa = 1.0
            j_wc = 1.0
            mass += ia + ic
            coordinate_a = aa - ac - self._reference_angle_a
        else:
            u = qc * self._local_axis_c
            rc = qc * (self._local_anchor_c - lcc)
            ra = qa * (self._local_anchor_a - lca)
            j_vac = u
            j_wc = rc.cross(u)
            j_wa = ra.cross(u)
            mass += mc + ma + ic * j_wc ** 2 + ia * j_wa ** 2

            pc = self._local_anchor_c - lcc
            pa = qc.mul_t(ra + (ca - cc))
            coordinate_a = (pa - pc).dot(self._local_axis_c)

        if self._revolute_b:
            j_vbd = Vec2()
            j_wb = 1.0
            j_wd = 1.0
            mass += ib + id_

            coordinate_b = ab - ad - self._reference_angle_b
        else:
            u = qd * self._local_axis_d
            rd = qd * (self._local_anchor_d - lcd)
            rb = qb * (self._local_anchor_b - lcb)
            j_vbd = ratio * u
            j_wd = ratio * rd.cross(u)
            j_wb = ratio * rb.cross(u)
            mass += ratio**2 * (md + mb) + id_ * j_wd ** 2 + ib * j_wb ** 2

            pd = self._local_anchor_d - lcd
            pb = qd.mul_t(rb + (cb - cd))
            coordinate_b = (pb - pd).dot(self._local_axis_d)

        C = (coordinate_a + ratio * coordinate_b) - self._constant
        impulse = 0.0
        if mass > 0.0:
            impulse = -C / mass

            Jcs = [j_vac, j_vbd] * 2
            Jas = [j_wa, j_wb, j_wc, j_wd]
            for k, (i, m, jc, ja) in enumerate(zip(indices, self._masses, Jcs, Jas)):
                c, a = positions[i]
                positions[i] = (c + m * impulse * jc, a + i * impulse * ja)
        
        return True # linear_error < LINEAR_SLOP # TODO_ERIN not implemented
