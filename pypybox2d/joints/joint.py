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

__all__ = ('Joint',
           'INACTIVE_LIMIT', 'AT_LOWER_LIMIT', 'AT_UPPER_LIMIT', 'EQUAL_LIMITS', 
           'ALLOWED_STRETCH')

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, property)
from ..settings import LINEAR_SLOP

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

