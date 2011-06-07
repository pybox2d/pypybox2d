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

__all__ = ('Jacobian', 
           'Joint', 'DistanceJoint', 'RevoluteJoint', 'FrictionJoint', 
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

class Jacobian(object):
    __slots__ = ['linear_a', 'linear_b', 'angular_a', 'angular_b']
    def __init__(self):
        self.linear_a = Vec2()
        self.linear_b = Vec2()
        self.angular_a = 0.0
        self.angular_b = 0.0

    def zero(self):
        self.linear_a.zero()
        self.linear_b.zero()
        self.angular_a = 0.0
        self.angular_b = 0.0

    def set(self, x1, a1, x2, a2):
        self.linear_a = Vec2(*x1)
        self.linear_b = Vec2(*x2)
        self.angular_a = a1
        self.angular_b = a2

    def compute(self, x1, a1, x2, a2):
        return self.linear_a.dot(x1) + self.angular_a * a1 + \
                self.linear_b.dot(x2) + self.angular_b * a2


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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * Vec2(self._impulse.x, self._impulse.y)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
        return inv_dt * self._impulse.z

    def _init_velocity_constraints(self, step):
        raise NotImplementedError # Implemented in subclass
    def _solve_velocity_constraints(self, step):
        raise NotImplementedError # Implemented in subclass
    def _solve_position_constraints(self, baumgarte):
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
    # m (v2 - v1) = lambda
    # v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
    # x2 = x1 + h * v2
    #                                                                           
    # 1-D mass-damper-spring system
    # m (v2 - v1) + h * d * v2 + h * k * 
    #                                                                           
    # C = norm(p2 - p1) - L
    # u = (p2 - p1) / norm(p2 - p1)
    # Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    # J = [-u -cross(r1, u) u cross(r2, u)]
    # K = J * invM * JT
    #   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return (inv_dt * self._impulse) * self._u

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
        return 0.0

    def _init_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        # Compute the effective mass matrix.
        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)
        self._u = b2._sweep.c + r2 - b1._sweep.c - r1

        # Handle singularity.
        length = self._u.length
        if length > LINEAR_SLOP:
            self._u *= 1.0 / length
        else:
            self._u = Vec2(0.0, 0.0)

        cr1u = r1.cross(self._u)
        cr2u = r2.cross(self._u)
        inv_mass = b1._inv_mass + b1._invI * cr1u * cr1u + b2._inv_mass + b2._invI * cr2u * cr2u

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
            gamma = step.dt * (d + step.dt * k)
            if gamma != 0.0:
                self._gamma = 1.0 / gamma
            else:
                self._gamma = 0.0
            self._bias = C * step.dt * k * self._gamma

            mass = inv_mass + self._gamma
            if mass != 0.0:
                self._mass = 1.0 / mass
            else:
                self._mass = 0.0

        if step.warm_starting:
            # Scale the impulse to support a variable time step.
            self._impulse *= step.dt_ratio

            P = self._impulse * self._u
            b1._linear_velocity -= b1._inv_mass * P
            b1._angular_velocity -= b1._invI * r1.cross(P)
            b2._linear_velocity += b2._inv_mass * P
            b2._angular_velocity += b2._invI * r2.cross(P)
        else:
            self._impulse = 0.0

    def _solve_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        # Cdot = dot(u, v + cross(w, r))
        v1 = b1._linear_velocity + scalar_cross(b1._angular_velocity, r1)
        v2 = b2._linear_velocity + scalar_cross(b2._angular_velocity, r2)
        Cdot = self._u.dot(v2 - v1)

        impulse = -self._mass * (Cdot + self._bias + self._gamma * self._impulse)
        self._impulse += impulse

        P = impulse * self._u
        b1._linear_velocity -= b1._inv_mass * P
        b1._angular_velocity -= b1._invI * r1.cross(P)
        b2._linear_velocity += b2._inv_mass * P
        b2._angular_velocity += b2._invI * r2.cross(P)

    def _solve_position_constraints(self, baumgarte):
        if self._frequency> 0.0:
            # There is no position correction for soft distance constraints.
            return True

        b1 = self._body_a
        b2 = self._body_b

        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        d = b2._sweep.c + r2 - b1._sweep.c - r1

        length = d.normalize()
        C = length - self._length

        impulse = -self._mass * clamp(C, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION)
        self._u = d
        P = impulse * self._u

        b1._sweep.c -= b1._inv_mass * P
        b1._sweep.a -= b1._invI * r1.cross(P)
        b2._sweep.c += b2._inv_mass * P
        b2._sweep.a += b2._invI * r2.cross(P)

        b1._synchronize_transform()
        b2._synchronize_transform()

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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * Vec2(*self._impulse[:2])

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
        return inv_dt * self._impulse.z

    def _init_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        if self._motor_enabled or self._limit_enabled:
            # You cannot create a rotation limit between bodies that
            # both have fixed rotation.
            assert(b1._invI > 0.0 or b2._invI > 0.0)

        # Compute the effective mass matrix.
        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        # J = [-I -r1_skew I r2_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
        #     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
        #     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

        m1, m2 = b1._inv_mass, b2._inv_mass
        i1, i2 = b1._invI, b2._invI

        # TODO: when rewriting Mat33 in C, these will need to be _col1, etc.
        self._mass.col1.x = m1 + m2 + r1.y * r1.y * i1 + r2.y * r2.y * i2
        self._mass.col2.x = -r1.y * r1.x * i1 - r2.y * r2.x * i2
        self._mass.col3.x = -r1.y * i1 - r2.y * i2
        self._mass.col1.y = self._mass.col2.x
        self._mass.col2.y = m1 + m2 + r1.x * r1.x * i1 + r2.x * r2.x * i2
        self._mass.col3.y = r1.x * i1 + r2.x * i2
        self._mass.col1.z = self._mass.col3.x
        self._mass.col2.z = self._mass.col3.y
        self._mass.col3.z = i1 + i2

        self._motor_mass = i1 + i2
        if self._motor_mass > 0.0:
            self._motor_mass = 1.0 / self._motor_mass

        if not self._motor_enabled:
            self._motor_impulse = 0.0

        if self._limit_enabled:
            joint_angle = b2._sweep.a - b1._sweep.a - self._reference_angle
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

            b1._linear_velocity -= m1 * P
            b1._angular_velocity -= i1 * (r1.cross(P) + self._motor_impulse + self._impulse.z)

            b2._linear_velocity += m2 * P
            b2._angular_velocity += i2 * (r2.cross(P) + self._motor_impulse + self._impulse.z)
        else:
            self._impulse.zero()
            self._motor_impulse = 0.0

    def _solve_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        v1 = b1._linear_velocity
        w1 = b1._angular_velocity
        v2 = b2._linear_velocity
        w2 = b2._angular_velocity

        m1, m2 = b1._inv_mass, b2._inv_mass
        i1, i2 = b1._invI, b2._invI

        # Solve motor constraint.
        if self._motor_enabled and self._limit_state != EQUAL_LIMITS:
            Cdot = w2 - w1 - self._motor_speed
            impulse = self._motor_mass * (-Cdot)
            old_impulse = self._motor_impulse
            max_impulse = step.dt * self._max_motor_torque
            self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
            impulse = self._motor_impulse - old_impulse

            w1 -= i1 * impulse
            w2 += i2 * impulse

        # Solve limit constraint.
        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT:
            # Compute the effective mass matrix.
            r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
            r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

            # Solve point-to-point constraint
            Cdot1 = v2 + scalar_cross(w2, r2) - v1 - scalar_cross(w1, r1)
            Cdot2 = w2 - w1
            Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

            impulse = self._mass.solve3x3(-Cdot)

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

            v1 -= m1 * P
            w1 -= i1 * (r1.cross(P) + impulse.z)

            v2 += m2 * P
            w2 += i2 * (r2.cross(P) + impulse.z)
        else:
            # Compute the effective mass matrix.
            r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
            r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

            # Solve point-to-point constraint
            Cdot = v2 + scalar_cross(w2, r2) - v1 - scalar_cross(w1, r1)
            impulse = self._mass.solve2x2(-Cdot)

            self._impulse.x += impulse.x
            self._impulse.y += impulse.y

            v1 -= m1 * impulse
            w1 -= i1 * r1.cross(impulse)

            v2 += m2 * impulse
            w2 += i2 * r2.cross(impulse)

        b1._angular_velocity = w1
        b2._angular_velocity = w2

    def _solve_position_constraints(self, baumgarte):
        b1 = self._body_a
        b2 = self._body_b

        # TODO_ERIN block solve with limit.
        angular_error = 0.0
        position_error = 0.0

        # Solve angular limit constraint.
        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT:
            angle = b2._sweep.a - b1._sweep.a - self._reference_angle
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

            b1._sweep.a -= b1._invI * limit_impulse
            b2._sweep.a += b2._invI * limit_impulse

            b1._synchronize_transform()
            b2._synchronize_transform()

        # Solve point-to-point constraint.
        # Compute the effective mass matrix.
        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        C = b2._sweep.c + r2 - b1._sweep.c - r1
        position_error = C.length

        inv_mass1, inv_mass2 = b1._inv_mass, b2._inv_mass
        inv_i1, inv_i2 = b1._invI, b2._invI

        # Handle large detachment.
        if C.length_squared > ALLOWED_STRETCH**2:
            # Use a particle solution (no rotation).
            u = Vec2(*C)
            u.normalize()
            m = inv_mass1 + inv_mass2
            if m > 0.0:
                m = 1.0 / m
            impulse = m * (-C)

            k_beta = 0.5
            b1._sweep.c -= k_beta * inv_mass1 * impulse
            b2._sweep.c += k_beta * inv_mass2 * impulse

            C = b2._sweep.c + r2 - b1._sweep.c - r1

        K1 = Mat22()
        K1._col1.x = inv_mass1 + inv_mass2;  K1._col2.x = 0.0
        K1._col1.y = 0.0;                    K1._col2.y = inv_mass1 + inv_mass2

        K2 = Mat22()
        K2._col1.x =  inv_i1 * r1.y * r1.y;  K2._col2.x = -inv_i1 * r1.x * r1.y
        K2._col1.y = -inv_i1 * r1.x * r1.y;  K2._col2.y =  inv_i1 * r1.x * r1.x

        K3 = Mat22()
        K3._col1.x =  inv_i2 * r2.y * r2.y;  K3._col2.x = -inv_i2 * r2.x * r2.y
        K3._col1.y = -inv_i2 * r2.x * r2.y;  K3._col2.y =  inv_i2 * r2.x * r2.x

        K = K1 + K2 + K3
        impulse = K.solve(-C)

        b1._sweep.c -= b1._inv_mass * impulse
        b1._sweep.a -= b1._invI * r1.cross(impulse)

        b2._sweep.c += b2._inv_mass * impulse
        b2._sweep.a += b2._invI * r2.cross(impulse)

        b1._synchronize_transform()
        b2._synchronize_transform()
        
        return position_error <= LINEAR_SLOP and angular_error <= ANGULAR_SLOP



class FrictionJoint(Joint):
    """
    Friction joint. This is used for top-down friction.
    It provides 2D translational friction and angular friction.
    """
    # Point-to-point constraint
    # Cdot = v2 - v1
    #      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    # J = [-I -r1_skew I r2_skew ]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)

    # Angle constraint
    # Cdot = w2 - w1
    # J = [0 0 -1 0 0 1]
    # K = invI1 + invI2
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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * self._linear_impulse

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

    def _init_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        # Compute the effective mass matrix.
        ra = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        rb = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        # J = [-I -r1_skew I r2_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        #     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        #     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        m_a, m_b = ba._inv_mass, bb._inv_mass
        i_a, i_b = ba._invI, bb._invI

        K1 = Mat22()
        K1._col1.x = m_a + m_b;  K1._col2.x = 0.0
        K1._col1.y = 0.0;        K1._col2.y = m_a + m_b

        K2 = Mat22()
        K2._col1.x =  i_a * ra.y * ra.y; K2._col2.x = -i_a * ra.x * ra.y
        K2._col1.y = -i_a * ra.x * ra.y; K2._col2.y =  i_a * ra.x * ra.x

        K3 = Mat22()
        K3._col1.x =  i_b * rb.y * rb.y; K3._col2.x = -i_b * rb.x * rb.y
        K3._col1.y = -i_b * rb.x * rb.y; K3._col2.y =  i_b * rb.x * rb.x

        K = K1 + K2 + K3
        self._linear_mass = K.inverse

        self._angular_mass = i_a + i_b
        if self._angular_mass > 0.0:
            self._angular_mass = 1.0 / self._angular_mass

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._linear_impulse *= step.dt_ratio
            self._angular_impulse *= step.dt_ratio

            P = Vec2(self._linear_impulse.x, self._linear_impulse.y)

            ba._linear_velocity -= m_a * P
            ba._angular_velocity -= i_a * (ra.cross(P) + self._angular_impulse)

            bb._linear_velocity += m_b * P
            bb._angular_velocity += i_b * (rb.cross(P) + self._angular_impulse)
        else:
            self._linear_impulse.zero()
            self._angular_impulse = 0.0

    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        v_a = ba._linear_velocity
        w_a = ba._angular_velocity
        v_b = bb._linear_velocity
        w_b = bb._angular_velocity

        m_a, m_b = ba._inv_mass, bb._inv_mass
        i_a, i_b = ba._invI, bb._invI

        ra = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        rb = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        # Solve angular friction
        Cdot = w_b - w_a
        impulse = -self._angular_mass * Cdot

        old_impulse = self._angular_impulse
        max_impulse = step.dt * self._max_torque
        self._angular_impulse = clamp(self._angular_impulse + impulse, -max_impulse, max_impulse)
        impulse = self._angular_impulse - old_impulse

        w_a -= i_a * impulse
        w_b += i_b * impulse

        # Solve linear friction
        Cdot = v_b + scalar_cross(w_b, rb) - v_a - scalar_cross(w_a, ra)

        impulse = -(self._linear_mass * Cdot)
        old_impulse = Vec2(*self._linear_impulse)
        self._linear_impulse += impulse

        max_impulse = step.dt * self._max_force

        if self._linear_impulse.length_squared > max_impulse ** 2:
            self._linear_impulse.normalize()
            self._linear_impulse *= max_impulse

        impulse = self._linear_impulse - old_impulse

        v_a -= m_a * impulse
        w_a -= i_a * ra.cross(impulse)

        v_b += m_b * impulse
        w_b += i_b * rb.cross(impulse)

        ba._angular_velocity = w_a
        bb._angular_velocity = w_b

    def _solve_position_constraints(self, baumgarte):
        return True

class PrismaticJoint(Joint):
    """
    A prismatic joint. This joint provides one degree of freedom: translation
    along an axis fixed in body1. Relative rotation is prevented. You can
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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * (self._impulse.x * self._perp + (self._motor_impulse + self._impulse.z) * self._axis)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

        ra = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        rb = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)
        p1 = ba._sweep.c + ra
        p2 = bb._sweep.c + rb
        d = p2 - p1
        axis = self._body_a.get_world_vector(self._local_x_axis);

        v1 = ba._linear_velocity
        v2 = bb._linear_velocity
        w1 = ba._angular_velocity
        w2 = bb._angular_velocity

        speed = d.dot(scalar_cross(w1, axis)) + axis.dot(v2 + scalar_cross(w2, rb) - v1 - scalar_cross(w1, ra))
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

    def _init_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        self._local_center_a = Vec2(*ba._sweep.local_center)
        self._local_center_b = Vec2(*bb._sweep.local_center)
        xf1 = ba._xf
        xf2 = bb._xf

        # Compute the effective mass matrix.
        r1 = xf1._rotation * (self._local_anchor_a - self._local_center_a)
        r2 = xf2._rotation * (self._local_anchor_b - self._local_center_b)
        d = bb._sweep.c + r2 - ba._sweep.c - r1

        m1 = self._inv_mass_a = ba._inv_mass
        i1 = self._inv_Ia = ba._invI
        m2 = self._inv_mass_b = bb._inv_mass
        i2 = self._inv_Ib = bb._invI

        # Compute motor Jacobian and effective mass.
        self._axis = xf1._rotation * self._local_x_axis
        a1 = self._a1 = (d + r1).cross(self._axis)
        a2 = self._a2 = r2.cross(self._axis)

        self._motor_mass = m1 + m2 + i1 * self._a1**2 + i2 * self._a2**2
        if self._motor_mass > 0.0:
            self._motor_mass = 1.0 / self._motor_mass

        # Prismatic constraint.
        self._perp = xf1._rotation * self._local_y_axis

        s1 = self._s1 = (d + r1).cross(self._perp)
        s2 = self._s2 = r2.cross(self._perp)

        k11 = m1 + m2 + i1 * (s1 ** 2) + i2 * (s2 ** 2)
        k12 = i1 * s1 + i2 * s2
        k13 = i1 * s1 * a1 + i2 * s2 * a2
        k22 = i1 + i2
        if k22 == 0.0:
            k22 = 1.0
        k23 = i1 * a1 + i2 * a2
        k33 = m1 + m2 + i1 * (a1 ** 2) + i2 * (a2 ** 2)

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

            P = ix * self._perp + (mi + iz) * self._axis
            L1 = ix * s1 + iy + (mi + iz) * a1
            L2 = ix * s2 + iy + (mi + iz) * a2

            ba._linear_velocity -= m1 * P
            ba._angular_velocity -= i1 * L1

            bb._linear_velocity += m2 * P
            bb._angular_velocity += i2 * L2
        else:
            self._impulse.zero()
            self._motor_impulse = 0.0

    def _solve_position_constraints(self, baumgarte):
        """This returns True if the position errors are within tolerance."""
        ba = self._body_a
        bb = self._body_b

        center_a = ba._sweep.c
        a1 = ba._sweep.a

        center_b = bb._sweep.c
        a2 = bb._sweep.a

        # Solve linear limit constraint.
        linear_error = 0.0
        angular_error = 0.0
        active = False
        C2 = 0.0

        R1 = Mat22()
        R2 = Mat22()
        R1.angle = a1
        R2.angle = a2

        r1 = R1 * (self._local_anchor_a - self._local_center_a)
        r2 = R2 * (self._local_anchor_b - self._local_center_b)
        d = center_b + r2 - center_a - r1

        if self._limit_enabled:
            self._axis = R1 * self._local_x_axis
            self._a1 = (d + r1).cross(self._axis)
            self._a2 = r2.cross(self._axis)

            translation = self._axis.dot(d)
            if abs(self._upper_limit - self._lower_limit) < 2.0 * LINEAR_SLOP:
                # Prevent large angular corrections
                C2 = clamp(translation, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION)
                linear_error = abs(translation)
                active = True
            elif translation <= self._lower_limit:
                # Prevent large linear corrections and allow some slop.
                C2 = clamp(translation - self._lower_limit + LINEAR_SLOP, -MAX_LINEAR_CORRECTION, 0.0)
                linear_error = self._lower_limit - translation
                active = True
            elif translation >= self._upper_limit:
                # Prevent large linear corrections and allow some slop.
                C2 = clamp(translation - self._upper_limit - LINEAR_SLOP, 0.0, MAX_LINEAR_CORRECTION)
                linear_error = translation - self._upper_limit
                active = True

        self._perp = R1 * self._local_y_axis

        s1 = self._s1 = (d + r1).cross(self._perp)
        s2 = self._s2 = (r2).cross(self._perp)

        C1 = Vec2(self._perp.dot(d), a2 - a1 - self._reference_angle)

        linear_error = max(linear_error, abs(C1.x))
        angular_error = abs(C1.y)

        m1 = self._inv_mass_a
        m2 = self._inv_mass_b
        i1 = self._inv_Ia
        i2 = self._inv_Ib

        if active:
            k11 = m1 + m2 + i1 * (s1 ** 2) + i2 * (s2 ** 2)
            k12 = i1 * s1 + i2 * s2
            k13 = i1 * s1 * self._a1 + i2 * s2 * self._a2
            k22 = i1 + i2
            if k22 == 0.0:
                k22 = 1.0
            k23 = i1 * self._a1 + i2 * self._a2
            k33 = m1 + m2 + i1 * (self._a1 ** 2) + i2 * (self._a2 ** 2)

            self._k = Mat33((k11, k12, k13), (k12, k22, k23), (k13, k23, k33))
            C = Vec3(C1.x, C1.y, C2)

            impulse = self._k.solve3x3(-C)
        else:
            k11 = m1 + m2 + i1 * s1 * s1 + i2 * s2 * s2
            k12 = i1 * s1 + i2 * s2
            k22 = i1 + i2
            if k22 == 0.0:
                k22 = 1.0

            self._k.col1 = Vec3(k11, k12, 0.0)
            self._k.col2 = Vec3(k12, k22, 0.0)

            impulse1 = self._k.solve2x2(-C1)
            impulse = (impulse1.x, impulse1.y, 0.0)

        ix, iy, iz = impulse
        P = ix * self._perp + iz * self._axis
        L1 = ix * s1 + iy + iz * self._a1
        L2 = ix * s2 + iy + iz * self._a2

        center_a -= self._inv_mass_a * P # modifies ba._sweep.c
        center_b += self._inv_mass_b * P
        a1 -= i1 * L1
        a2 += i2 * L2

        ba._sweep.a = a1
        bb._sweep.a = a2
        ba._synchronize_transform()
        bb._synchronize_transform()
        return (linear_error <= LINEAR_SLOP) and (angular_error <= ANGULAR_SLOP)
        
    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        v1 = ba._linear_velocity # not copied, so modified when v1 modified
        w1 = ba._angular_velocity
        v2 = bb._linear_velocity
        w2 = bb._angular_velocity

        # Solve linear motor constraint.
        if self._motor_enabled and self._limit_state != EQUAL_LIMITS:
            Cdot = self._axis.dot(v2 - v1) + self._a2 * w2 - self._a1 * w1
            impulse = self._motor_mass * (self._motor_speed - Cdot)
            old_impulse = self._motor_impulse
            max_impulse = step.dt * self._max_motor_force
            self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
            impulse = self._motor_impulse - old_impulse

            P = impulse * self._axis
            L1 = impulse * self._a1
            L2 = impulse * self._a2

            v1 -= self._inv_mass_a * P
            w1 -= self._inv_Ia * L1

            v2 += self._inv_mass_b * P
            w2 += self._inv_Ib * L2

        Cdot1 = Vec2(self._perp.dot(v2 - v1) + self._s2 * w2 - self._s1 * w1, 
                     w2 - w1)

        if self._limit_enabled and self._limit_state != INACTIVE_LIMIT:
            # Solve prismatic and limit constraint in block form.
            Cdot2 = self._axis.dot(v2 - v1) + self._a2 * w2 - self._a1 * w1
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

            P = df.x * self._perp + df.z * self._axis
            L1 = df.x * self._s1 + df.y + df.z * self._a1
            L2 = df.x * self._s2 + df.y + df.z * self._a2

            v1 -= self._inv_mass_a * P
            w1 -= self._inv_Ia * L1

            v2 += self._inv_mass_b * P
            w2 += self._inv_Ib * L2
        else:
            # Limit is inactive, just solve the prismatic constraint in block form.
            df = self._k.solve2x2(-Cdot1)
            self._impulse.x += df.x
            self._impulse.y += df.y

            P = df.x * self._perp
            L1 = df.x * self._s1 + df.y
            L2 = df.x * self._s2 + df.y

            v1 -= self._inv_mass_a * P
            w1 -= self._inv_Ia * L1

            v2 += self._inv_mass_b * P
            w2 += self._inv_Ib * L2

        ba._angular_velocity = w1
        bb._angular_velocity = w2



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
    # Cdot = v2 - v1
    #      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    # J = [-I -r1_skew I r2_skew ]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)

    # Angle constraint
    # C = angle2 - angle1 - referenceAngle
    # Cdot = w2 - w1
    # J = [0 0 -1 0 0 1]
    # K = invI1 + invI2

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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * Vec2(self._impulse.x, self._impulse.y)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
        return inv_dt * self._impulse.z

    @property
    def anchor_a(self):
        """Get the anchor point on body_a in world coordinates"""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """Get the anchor point on body_b in world coordinates"""
        return self._body_b.get_world_point(self._local_anchor_b)

    def _init_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        # Compute the effective mass matrix.
        r_a = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        r_b = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        # J = [-I -r1_skew I r2_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        #     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        #     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        m_a, m_b = ba._inv_mass, bb._inv_mass
        i_a, i_b = ba._invI, bb._invI

        self._mass.col1.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b
        self._mass.col2.x = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b
        self._mass.col3.x = -r_a.y * i_a - r_b.y * i_b
        self._mass.col1.y = self._mass.col2.x
        self._mass.col2.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b
        self._mass.col3.y = r_a.x * i_a + r_b.x * i_b
        self._mass.col1.z = self._mass.col3.x
        self._mass.col2.z = self._mass.col3.y
        self._mass.col3.z = i_a + i_b

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._impulse *= step.dt_ratio

            P = Vec2(self._impulse.x, self._impulse.y)

            ba._linear_velocity -= m_a * P
            ba._angular_velocity -= i_a * (r_a.cross(P) + self._impulse.z)

            bb._linear_velocity += m_b * P
            bb._angular_velocity += i_b * (r_b.cross(P) + self._impulse.z)
        else:
            self._impulse.zero()
        
    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        v_a = ba._linear_velocity
        w_a = ba._angular_velocity
        v_b = bb._linear_velocity
        w_b = bb._angular_velocity

        m_a, m_b = ba._inv_mass, bb._inv_mass
        i_a, i_b = ba._invI, bb._invI

        r_a = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        r_b = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        # Solve point-to-point constraint
        Cdot1 = v_b + scalar_cross(w_b, r_b) - v_a - scalar_cross(w_a, r_a)
        Cdot2 = w_b - w_a
        Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

        impulse = self._mass.solve3x3(-Cdot)
        self._impulse += impulse

        P = Vec2(impulse.x, impulse.y)

        v_a -= m_a * P
        w_a -= i_a * (r_a.cross(P) + impulse.z)

        v_b += m_b * P
        w_b += i_b * (r_b.cross(P) + impulse.z)

        ba._linear_velocity = v_a
        ba._angular_velocity = w_a
        bb._linear_velocity = v_b
        bb._angular_velocity = w_b

    def _solve_position_constraints(self, baumgarte):
        """This returns true if the position errors are within tolerance."""
        ba = self._body_a
        bb = self._body_b

        m_a, m_b = ba._inv_mass, bb._inv_mass
        i_a, i_b = ba._invI, bb._invI

        r_a = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        r_b = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        C1 = bb._sweep.c + r_b - ba._sweep.c - r_a
        C2 = bb._sweep.a - ba._sweep.a - self._reference_angle

        # Handle large detachment.
        position_error = C1.length
        angular_error = abs(C2)
        if position_error > ALLOWED_STRETCH:
            i_a *= 1.0
            i_b *= 1.0

        self._mass.col1.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b
        self._mass.col2.x = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b
        self._mass.col3.x = -r_a.y * i_a - r_b.y * i_b
        self._mass.col1.y = self._mass.col2.x
        self._mass.col2.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b
        self._mass.col3.y = r_a.x * i_a + r_b.x * i_b
        self._mass.col1.z = self._mass.col3.x
        self._mass.col2.z = self._mass.col3.y
        self._mass.col3.z = i_a + i_b

        C = Vec3(C1.x, C1.y, C2)

        impulse = self._mass.solve3x3(-C)

        P = Vec2(impulse.x, impulse.y)

        ba._sweep.c -= m_a * P
        ba._sweep.a -= i_a * (r_a.cross(P) + impulse.z)

        bb._sweep.c += m_b * P
        bb._sweep.a += i_b * (r_b.cross(P) + impulse.z)

        ba._synchronize_transform()
        bb._synchronize_transform()

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
                         self._local_anchorb, self._max_length, 
                         self._collide_connected)


    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return (inv_dt * self._impulse) * self._u

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
        return 0.0

    @property
    def limit_state(self):
        """The status of the limit (INACTIVE_LIMIT, AT_UPPER_LIMIT)"""
        return self._limit_state

    @property
    def max_length(self):
        """(Read-only) Maximum separation/rope length"""
        return self._max_length

    def _init_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        self._ra = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        self._rb = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        # Rope axis
        self._u = bb._sweep.c + self._rb - ba._sweep.c - self._ra

        self._length = self._u.length

        C = self._length - self._max_length
        if C > 0.0:
            self._state = AT_UPPER_LIMIT
        else:
            self._state = INACTIVE_LIMIT

        if self._length > LINEAR_SLOP:
            self._u *= 1.0 / self._length
        else:
            self._u.zero()
            self._mass = 0.0
            self._impulse = 0.0
            return

        # Compute effective mass.
        cra = self._ra.cross(self._u)
        crb = self._rb.cross(self._u)
        inv_mass = ba._inv_mass + ba._invI * cra * cra + bb._inv_mass + bb._invI * crb * crb

        if inv_mass != 0.0:
            self._mass = 1.0 / inv_mass
        else:
            self._mass = 0.0

        if step.warm_starting:
            # Scale the impulse to support a variable time step.
            self._impulse *= step.dt_ratio

            P = self._impulse * self._u
            ba._linear_velocity -= ba._inv_mass * P
            ba._angular_velocity -= ba._invI * self._ra.cross(P)
            bb._linear_velocity += bb._inv_mass * P
            bb._angular_velocity += bb._invI * self._rb.cross(P)
        else:
            self._impulse = 0.0

    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        # Cdot = dot(u, v + cross(w, r))
        v_a = ba._linear_velocity + scalar_cross(ba._angular_velocity, self._ra)
        v_b = bb._linear_velocity + scalar_cross(bb._angular_velocity, self._rb)
        C = self._length - self._max_length
        Cdot = self._u.dot(v_b - v_a)

        # Predictive constraint.
        if C < 0.0:
            Cdot += step.inv_dt * C

        impulse = -self._mass * Cdot
        old_impulse = self._impulse
        self._impulse = min(0.0, self._impulse + impulse)
        impulse = self._impulse - old_impulse

        P = impulse * self._u
        ba._linear_velocity -= ba._inv_mass * P
        ba._angular_velocity -= ba._invI * self._ra.cross(P)
        bb._linear_velocity += bb._inv_mass * P
        bb._angular_velocity += bb._invI * self._rb.cross(P)
        
    
    def _solve_position_constraints(self, baumgarte):
        """This returns true if the position errors are within tolerance."""
        ba = self._body_a
        bb = self._body_b

        ra = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
        rb = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)

        u = bb._sweep.c + rb - ba._sweep.c - ra

        length = u.normalize()
        C = length - self._max_length

        C = clamp(C, 0.0, MAX_LINEAR_CORRECTION)

        impulse = -self._mass * C
        P = impulse * u

        ba._sweep.c -= ba._inv_mass * P
        ba._sweep.a -= ba._invI * ra.cross(P)
        bb._sweep.c += bb._inv_mass * P
        bb._sweep.a += bb._invI * rb.cross(P)

        ba._synchronize_transform()
        bb._synchronize_transform()

        return length - self._max_length < LINEAR_SLOP


class WheelJoint(Joint):
    """
    A wheel joint. This joint provides two degrees of freedom: translation
    along an axis fixed in body1 and rotation in the plane. You can use a
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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * (self._impulse * self._ay + self._spring_impulse * self._ax)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

    def _init_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        self._local_center_a = ba._sweep.local_center
        self._local_center_b = bb._sweep.local_center

        xf_a = ba._xf
        xf_b = bb._xf

        # Compute the effective masses.
        ra = xf_a._rotation * (self._local_anchor_a - self._local_center_a)
        rb = xf_b._rotation * (self._local_anchor_b - self._local_center_b)
        d = bb._sweep.c + rb - ba._sweep.c - ra

        self._inv_mass_a = ba._inv_mass
        self._inv_i_a = ba._invI
        self._inv_mass_b = bb._inv_mass
        self._inv_i_b = bb._invI

        # Point to line constraint
        self._ay = xf_a._rotation * (self._local_y_axis)
        self._s_ay = (d + ra).cross(self._ay)
        self._s_by = rb.cross(self._ay)

        self._mass = self._inv_mass_a + self._inv_mass_b + self._inv_i_a * self._s_ay * self._s_ay + self._inv_i_b * self._s_by * self._s_by

        if self._mass > 0.0:
            self._mass = 1.0 / self._mass

        # Spring constraint
        self._spring_mass = 0.0
        if self._frequency > 0.0:
            self._ax = xf_a._rotation * self._local_x_axis
            self._s_ax = (d + ra).cross(self._ax)
            self._s_bx = rb.cross(self._ax)

            inv_mass = self._inv_mass_a + self._inv_mass_b + self._inv_i_a * self._s_ax * self._s_ax + self._inv_i_b * self._s_bx * self._s_bx

            if inv_mass > 0.0:
                self._spring_mass = 1.0 / inv_mass

                C = d.dot(self._ax)

                # Frequency
                omega = 2.0 * PI * self._frequency

                # Damping coefficient
                d = 2.0 * self._spring_mass * self._damping_ratio * omega

                # Spring stiffness
                k = self._spring_mass * omega * omega

                # magic formulas
                self._gamma = step.dt * (d + step.dt * k)
                if self._gamma > 0.0:
                    self._gamma = 1.0 / self._gamma

                self._bias = C * step.dt * k * self._gamma

                self._spring_mass = inv_mass + self._gamma
                if self._spring_mass > 0.0:
                    self._spring_mass = 1.0 / self._spring_mass
        else:
            self._spring_impulse = 0.0
            self._spring_mass = 0.0

        # Rotational motor
        if self._motor_enabled:
            self._motor_mass = self._inv_i_a + self._inv_i_b
            if self._motor_mass > 0.0:
                self._motor_mass = 1.0 / self._motor_mass
        else:
            self._motor_mass = 0.0
            self._motor_impulse = 0.0

        if step.warm_starting:
            # Account for variable time step.
            self._impulse *= step.dt_ratio
            self._spring_impulse *= step.dt_ratio
            self._motor_impulse *= step.dt_ratio

            P = self._impulse * self._ay + self._spring_impulse * self._ax
            L_a = self._impulse * self._s_ay + self._spring_impulse * self._s_ax + self._motor_impulse
            L_b = self._impulse * self._s_by + self._spring_impulse * self._s_bx + self._motor_impulse

            ba._linear_velocity -= self._inv_mass_a * P
            ba._angular_velocity -= self._inv_i_a * L_a

            bb._linear_velocity += self._inv_mass_b * P
            bb._angular_velocity += self._inv_i_b * L_b
        else:
            self._impulse = 0.0
            self._spring_impulse = 0.0
            self._motor_impulse = 0.0


    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        v_a = ba._linear_velocity
        w_a = ba._angular_velocity
        v_b = bb._linear_velocity
        w_b = bb._angular_velocity

        # Solve spring constraint
        Cdot = self._ax.dot(v_b - v_a) + self._s_bx * w_b - self._s_ax * w_a
        impulse = -self._spring_mass * (Cdot + self._bias + self._gamma * self._spring_impulse)
        self._spring_impulse += impulse

        P = impulse * self._ax
        L_a = impulse * self._s_ax
        L_b = impulse * self._s_bx

        v_a -= self._inv_mass_a * P
        w_a -= self._inv_i_a * L_a

        v_b += self._inv_mass_b * P
        w_b += self._inv_i_b * L_b

        # Solve rotational motor constraint
        Cdot = w_b - w_a - self._motor_speed
        impulse = -self._motor_mass * Cdot

        old_impulse = self._motor_impulse
        max_impulse = step.dt * self._max_motor_torque
        self._motor_impulse = clamp(self._motor_impulse + impulse, -max_impulse, max_impulse)
        impulse = self._motor_impulse - old_impulse

        w_a -= self._inv_i_a * impulse
        w_b += self._inv_i_b * impulse

        # Solve point to line constraint
        Cdot = self._ay.dot(v_b - v_a) + self._s_by * w_b - self._s_ay * w_a
        impulse = self._mass * (-Cdot)
        self._impulse += impulse

        P = impulse * self._ay
        L_a = impulse * self._s_ay
        L_b = impulse * self._s_by

        v_a -= self._inv_mass_a * P
        w_a -= self._inv_i_a * L_a

        v_b += self._inv_mass_b * P
        w_b += self._inv_i_b * L_b

        ba._linear_velocity = v_a
        ba._angular_velocity = w_a
        bb._linear_velocity = v_b
        bb._angular_velocity = w_b

    def _solve_position_constraints(self, baumgarte):
        """This returns true if the position errors are within tolerance."""
        ba = self._body_a
        bb = self._body_b

        x_a = ba._sweep.c
        angle_a = ba._sweep.a

        x_b = bb._sweep.c
        angle_b = bb._sweep.a

        R_a = Mat22()
        R_b = Mat22()
        R_a.angle = angle_a
        R_b.angle = angle_b

        ra = ba._xf._rotation * (self._local_anchor_a - self._local_center_a)
        rb = bb._xf._rotation * (self._local_anchor_b - self._local_center_b)
        d = x_b + rb - x_a - ra

        ay = R_a * self._local_y_axis

        s_ay = (d + ra).cross(ay)
        s_by = rb.cross(ay)

        C = d.dot(ay)

        k = self._inv_mass_a + self._inv_mass_b + self._inv_i_a * self._s_ay * self._s_ay + self._inv_i_b * self._s_by * self._s_by

        if k != 0.0:
            impulse = - C / k
        else:
            impulse = 0.0

        P = impulse * ay
        L_a = impulse * s_ay
        L_b = impulse * s_by

        x_a -= self._inv_mass_a * P
        angle_a -= self._inv_i_a * L_a
        x_b += self._inv_mass_b * P
        angle_b += self._inv_i_b * L_b

        ba._sweep.a = angle_a
        bb._sweep.a = angle_b
        ba._synchronize_transform()
        bb._synchronize_transform()

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
        self._local_anchor = body.get_local_point(target)
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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * self._impulse

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

    def _init_velocity_constraints(self, step):
        b = self._body_b
        mass = b.mass

        # Frequency
        omega = 2.0 * PI * self._frequency

        # Damping coefficient
        d = 2.0 * mass * self._damping_ratio * omega

        # Spring stiffness
        k = mass * (omega * omega)

        # magic formulas
        # gamma has units of inverse mass.
        # beta has units of inverse time.
        assert(d + step.dt * k > EPSILON)
        self._gamma = step.dt * (d + step.dt * k)
        if self._gamma != 0.0:
            self._gamma = 1.0 / self._gamma
        self._beta = step.dt * k * self._gamma

        # Compute the effective mass matrix.
        r = b._xf._rotation * (self._local_anchor - b._sweep.local_center)

        # K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        #      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        #        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        inv_mass = b._inv_mass
        inv_i = b._invI

        K1 = Mat22()
        K1._col1.x = inv_mass;	K1._col2.x = 0.0
        K1._col1.y = 0.0;		K1._col2.y = inv_mass

        K2 = Mat22()
        K2._col1.x =  inv_i * r.y * r.y;	K2._col2.x = -inv_i * r.x * r.y
        K2._col1.y = -inv_i * r.x * r.y;	K2._col2.y =  inv_i * r.x * r.x

        K = K1 + K2
        K._col1.x += self._gamma
        K._col2.y += self._gamma

        self._mass = K.inverse

        self.__c = b._sweep.c + r - self._target

        # Cheat with some damping
        b._angular_velocity *= 0.98

        # Warm starting.
        self._impulse *= step.dt_ratio
        b._linear_velocity += inv_mass * self._impulse
        b._angular_velocity += inv_i * r.cross(self._impulse)
        
    def _solve_velocity_constraints(self, step):
        b = self._body_b

        r = b._xf._rotation * (self._local_anchor - b._sweep.local_center)

        # Cdot = v + cross(w, r)
        Cdot = b._linear_velocity + scalar_cross(b._angular_velocity, r)
        impulse = self._mass * (-(Cdot + self._beta * self.__c + self._gamma * self._impulse))

        old_impulse = Vec2(*self._impulse)
        self._impulse += impulse
        max_impulse = step.dt * self._max_force
        if self._impulse.length_squared > max_impulse * max_impulse:
            self._impulse *= max_impulse / self._impulse.length
        impulse = self._impulse - old_impulse

        b._linear_velocity += b._inv_mass * impulse
        b._angular_velocity += b._invI * r.cross(impulse)
    
    def _solve_position_constraints(self, baumgarte):
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
    # Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
    # J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
    # K = J * invM * JT
    #   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
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
        self._pulley_mass = 0.0

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
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * self._impulse * self._u2

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

    def _init_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        p1 = b1._sweep.c + r1
        p2 = b2._sweep.c + r2

        s1 = self._ground_anchor_a
        s2 = self._ground_anchor_b

        # Get the pulley axes.
        self._u1 = p1 - s1
        self._u2 = p2 - s2

        length1 = self._u1.length
        length2 = self._u2.length

        if length1 > 10.0 * LINEAR_SLOP:
            self._u1 *= 1.0 / length1
        else:
            self._u1.zero()

        if length2 > 10.0 * LINEAR_SLOP:
            self._u2 *= 1.0 / length2
        else:
            self._u2.zero()

        # Compute effective mass.
        cr1u1 = r1.cross(self._u1)
        cr2u2 = r2.cross(self._u2)

        m1 = b1._inv_mass + b1._invI * cr1u1 * cr1u1
        m2 = b2._inv_mass + b2._invI * cr2u2 * cr2u2

        self._pulley_mass = m1 + self._ratio * self._ratio * m2

        if self._pulley_mass > 0.0:
            self._pulley_mass = 1.0 / self._pulley_mass

        if step.warm_starting:
            # Scale impulses to support variable time steps.
            self._impulse *= step.dt_ratio

            # Warm starting.
            P1 = -(self._impulse) * self._u1
            P2 = (-self._ratio * self._impulse) * self._u2
            b1._linear_velocity += b1._inv_mass * P1
            b1._angular_velocity += b1._invI * r1.cross(P1)
            b2._linear_velocity += b2._inv_mass * P2
            b2._angular_velocity += b2._invI * r2.cross(P2)
        else:
            self._impulse = 0.0

    def _solve_velocity_constraints(self, step):
        b1 = self._body_a
        b2 = self._body_b

        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        v1 = b1._linear_velocity + scalar_cross(b1._angular_velocity, r1)
        v2 = b2._linear_velocity + scalar_cross(b2._angular_velocity, r2)

        Cdot = -self._u1.dot(v1) - self._ratio * self._u2.dot(v2)
        impulse = self._pulley_mass * (-Cdot)
        self._impulse += impulse

        P1 = -impulse * self._u1
        P2 = -self._ratio * impulse * self._u2
        b1._linear_velocity += b1._inv_mass * P1
        b1._angular_velocity += b1._invI * r1.cross(P1)
        b2._linear_velocity += b2._inv_mass * P2
        b2._angular_velocity += b2._invI * r2.cross(P2)

    def _solve_position_constraints(self, baumgarte):
        """This returns true if the position errors are within tolerance."""
        b1 = self._body_a
        b2 = self._body_b

        s1 = self._ground_anchor_a
        s2 = self._ground_anchor_b

        r1 = b1._xf._rotation * (self._local_anchor_a - b1._sweep.local_center)
        r2 = b2._xf._rotation * (self._local_anchor_b - b2._sweep.local_center)

        p1 = b1._sweep.c + r1
        p2 = b2._sweep.c + r2

        # Get the pulley axes.
        u1 = p1 - s1
        u2 = p2 - s2

        length1 = u1.length
        length2 = u2.length

        if length1 > 10.0 * LINEAR_SLOP:
            u1 *= 1.0 / length1
        else:
            u1.zero()

        if length2 > 10.0 * LINEAR_SLOP:
            u2 *= 1.0 / length2
        else:
            u2.zero()

        # Compute effective mass.
        cr1u1 = r1.cross(u1)
        cr2u2 = r2.cross(u2)

        m1 = b1._inv_mass + b1._invI * cr1u1 * cr1u1
        m2 = b2._inv_mass + b2._invI * cr2u2 * cr2u2

        mass = m1 + self._ratio * self._ratio * m2

        if mass > 0.0:
            mass = 1.0 / mass

        C = self._constant - length1 - self._ratio * length2
        linear_error = abs(C)

        impulse = -mass * C

        P1 = -impulse * u1
        P2 = -self._ratio * impulse * u2

        b1._sweep.c += b1._inv_mass * P1
        b1._sweep.a += b1._invI * r1.cross(P1)
        b2._sweep.c += b2._inv_mass * P2
        b2._sweep.a += b2._invI * r2.cross(P2)

        b1._synchronize_transform()
        b2._synchronize_transform()

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
    #   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
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

        self._revolute_a = None
        self._revolute_b = None
        self._prismatic_a = None
        self._prismatic_b = None

        self._joint_a = joint_a
        self._joint_b = joint_b

        self._ground_a = joint_a.body_a
        self._body_a = joint_a.body_b
        if isinstance(joint_a, RevoluteJoint):
            self._revolute_a = joint_a
            coordinate_a = joint_a.joint_angle
        else:
            self._prismatic_a = joint_a
            coordinate_a = joint_a.joint_translation

        self._ground_anchor_a = Vec2(*joint_a._local_anchor_a)
        self._local_anchor_a = Vec2(*joint_a._local_anchor_b)
    
        self._ground_b = joint_a.body_a
        self._body_b = joint_a.body_b
        if isinstance(joint_b, RevoluteJoint):
            self._revolute_b = joint_b
            coordinate_b = joint_b.joint_angle
        else:
            self._prismatic_b = joint_b
            coordinate_b = joint_b.joint_translation

        self._ground_anchor_b = Vec2(*joint_b._local_anchor_a)
        self._local_anchor_b = Vec2(*joint_b._local_anchor_b)
        
        self._ratio = ratio
        self._constant = coordinate_a + self._ratio * coordinate_b
        self._impulse = 0.0
        self._j = Jacobian()

    def __copy__(self):
        return GearJoint(self._joint_a, self._joint_b, self._ratio)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body2 at the joint anchor in Newtons."""
        return inv_dt * (self._impulse * self._j.linear_b) # TODO_ERIN not tested

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body2 in N*m."""
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

    def _init_velocity_constraints(self, step):
        g1 = self._ground_a
        g2 = self._ground_b
        ba = self._body_a
        bb = self._body_b

        K = 0.0
        self._j.zero()

        if self._revolute_a is not None:
            self._j.angular_a = -1.0
            K += ba._invI
        else:
            ug = g1._xf._rotation * self._prismatic_a._local_x_axis
            r = ba._xf._rotation * (self._local_anchor_a - ba._sweep.local_center)
            crug = r.cross(ug)
            self._j.linear_a = -ug
            self._j.angular_a = -crug
            K += ba._inv_mass + ba._invI * (crug ** 2)

        if self._revolute_b is not None:
            self._j.angular_b = -self._ratio
            K += (self._ratio ** 2) * bb._invI
        else:
            ug = g2._xf._rotation * self._prismatic_b._local_x_axis
            r = bb._xf._rotation * (self._local_anchor_b - bb._sweep.local_center)
            crug = r.cross(ug)
            self._j.linear_b = -self._ratio * ug
            self._j.angular_b = -self._ratio * crug
            K += self._ratio * self._ratio * (bb._inv_mass + bb._invI * (crug ** 2))

        # Compute effective mass.
        if K > 0.0:
            self._mass = 1.0 / K
        else:
            self._mass = 0.0

        if step.warm_starting:
            # Warm starting.
            ba._linear_velocity += ba._inv_mass * self._impulse * self._j.linear_a
            ba._angular_velocity += ba._invI * self._impulse * self._j.angular_a
            bb._linear_velocity += bb._inv_mass * self._impulse * self._j.linear_b
            bb._angular_velocity += bb._invI * self._impulse * self._j.angular_b
        else:
            self._impulse = 0.0

    def _solve_velocity_constraints(self, step):
        ba = self._body_a
        bb = self._body_b

        Cdot = self._j.compute(ba._linear_velocity, ba._angular_velocity,
                               bb._linear_velocity, bb._angular_velocity)

        impulse = self._mass * (-Cdot)
        self._impulse += impulse

        ba._linear_velocity += ba._inv_mass * impulse * self._j.linear_a
        ba._angular_velocity += ba._invI * impulse * self._j.angular_a
        bb._linear_velocity += bb._inv_mass * impulse * self._j.linear_b
        bb._angular_velocity += bb._invI * impulse * self._j.angular_b

    def _solve_position_constraints(self, baumgarte):
        ba = self._body_a
        bb = self._body_b

        if self._revolute_a is not None:
            coordinate1 = self._revolute_a.joint_angle
        else:
            coordinate1 = self._prismatic_a.joint_translation

        if self._revolute_b is not None:
            coordinate2 = self._revolute_b.joint_angle
        else:
            coordinate2 = self._prismatic_b.joint_translation

        C = self._constant - (coordinate1 + self._ratio * coordinate2)

        impulse = self._mass * (-C)

        ba._sweep.c += ba._inv_mass * impulse * self._j.linear_a
        ba._sweep.a += ba._invI * impulse * self._j.angular_a
        bb._sweep.c += bb._inv_mass * impulse * self._j.linear_b
        bb._sweep.a += bb._invI * impulse * self._j.angular_b

        ba._synchronize_transform()
        bb._synchronize_transform()
        return True # linear_error < LINEAR_SLOP # TODO_ERIN not implemented
