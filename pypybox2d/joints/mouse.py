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

__all__ = ('MouseJoint', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (PI, Vec2, Mat22, scalar_cross, is_valid_float, property)
from ..settings import EPSILON
from .joint import Joint

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
        k = mass * (omega ** 2)

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
        K.col1 = Vec2(mb + ib * rb.y ** 2 + self._gamma,
                      -ib * rb.x * rb.y)
        K.col2 = Vec2(K.col1.y,
                      mb + ib * rb.x ** 2 + self._gamma)

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
            self._impulse = Vec2()

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

        old_impulse = self._impulse
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

