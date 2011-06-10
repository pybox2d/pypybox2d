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

__all__ = ('FixtureProxy', 'Fixture')
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import (AABB, EmptyFixtureError, is_valid_float, property)

class FixtureProxy(object):
    __slots__=['aabb', 'fixture', 'child_index', 'proxy_ref', '__weakref__']
    def __init__(self, fixture=None, aabb=None, child_index=0):
        self.aabb = copy(aabb)
        self.fixture = fixture
        self.child_index = child_index
        self.proxy_ref = None

    def __repr__(self):
        return 'FixtureProxy(fixture=%s, aabb=%s, child_index=%s)' % \
                (self.fixture, self.aabb, self.child_index)

class Fixture(object):
    """
    A fixture is used to attach a shape to a body for collision detection. A fixture
    inherits its transform from its parent. Fixtures hold additional non-geometric data
    such as friction, collision filters, etc.

    Fixtures are created via Body.create_fixture.
    @warning you cannot reuse fixtures.
    """
    __slots__ = ['_shape', '_friction', '_restitution', '_density', '_sensor',
                 '_proxies', '_body', '_category_bits', '_mask_bits', '_group_index', 'user_data']
    # TODO slots for debugging only
    def __init__(self, shape=None, friction=0.2, restitution=0.0, density=0.0, 
                    category_bits=0x0001, mask_bits=0xFFFF, group_index=0,
                    sensor=False, user_data=None, body=None):
        self._friction = friction
        self._restitution = restitution
        self._density = density
        self._sensor = sensor
        self._proxies = []

        self._category_bits = category_bits
        self._mask_bits = mask_bits
        self._group_index = group_index
        self._shape = copy(shape)
        self.user_data = user_data
        self._attach_to_body(body)

    def _attach_to_body(self, body):
        if body is not None and self._shape is None:
            raise EmptyFixtureError('Cannot attach empty fixture to a body')
        self._body = body

    def __copy__(self):
        return Fixture(shape=self._shape, 
                        friction=self._friction,
                        restitution=self._restitution,
                        density=self._density,
                        category_bits=self._category_bits,
                        mask_bits=self._mask_bits,
                        group_index=self._group_index,
                        sensor=self._sensor,
                        user_data=self.user_data)
    
    def __repr__(self):
        return 'Fixture(shape=%s, friction=%g, restitution=%g, density=%g, category_bits=0x%x, mask_bits=0x%x, group_index=%d, sensor=%s, user_data=%s)' % \
                (self.shape, self.friction, self.restitution, self.density, self.category_bits, self.mask_bits, self.group_index, self.sensor, repr(self.user_data))

    #--- start properties---
    @property
    def mass_data(self):
        """
        Get the mass data for this fixture. The mass data is based on the density and
        the shape. The rotational inertia is about the shape's origin. This operation
        may be expensive.
        """
        return self._shape.compute_mass(self._density)

    @property
    def body(self):
        """The body this fixture is attached to"""
        return self._body

    @property
    def shape(self):
        """
        The shape, this must be set to be attached to a body. The shape is cloned 
        on creation.
        
        You can modify the child shape, however you should not change the
        number of vertices because this will crash some collision caching mechanisms.
        Manipulating the shape may lead to non-physical behavior.
        """
        return self._shape

    @shape.setter
    def shape(self, shape):
        self._shape=shape
    
    @property
    def friction(self):
        """
        The friction coefficient, usually in the range [0,1].
        
        Setting this will not update the existing contacts.
        """
        return self._friction
    @friction.setter
    def friction(self, friction):
        self._friction = friction

    @property
    def restitution(self):
        """
        The restitution (elasticity) usually in the range [0,1].
        
        Setting this will not update the existing contacts.
        """
        return self._restitution
    @restitution.setter
    def restitution(self, restitution):
        self._restitution = restitution

    @property
    def density(self):
        """
        The density, usually in kg/m^2.

        Setting the density of this fixture will _not_ automatically adjust the mass
        of the body. You must call Body.reset_mass_data to update the body's mass.
        """
        return self._density
    @density.setter
    def density(self, density):
        if not is_valid_float(density) or density < 0:
            raise ValueError('Invalid density')

        self._density = density

    @property
    def sensor(self):
        """
        A sensor shape collects contact information but never 
        generates a collision response. It is a non-solid.
        """
        return self._sensor
    @sensor.setter
    def sensor(self, sensor):
        self._sensor = sensor

    @property
    def category_bits(self):
        """
        The collision category bits. Normally you would just set one bit.
        
        Setting this will not update contacts until the next time 
        step when either parent body is active and awake. Automatically 
        calls refilter.
        """
        return self._category_bits

    @category_bits.setter
    def category_bits(self, category_bits):
        self._category_bits = category_bits
        self._refilter()

    @property
    def mask_bits(self):
        """
        The collision mask bits. This states the categories that this
        shape would accept for collision.

        Setting this will not update contacts until the next time 
        step when either parent body is active and awake. Automatically 
        calls refilter.
        """
        return self._mask_bits

    @mask_bits.setter
    def mask_bits(self, mask_bits):
        self._mask_bits = mask_bits
        self._refilter()

    @property
    def group_index(self):
        """
        Collision groups allow a certain group of objects to never collide (negative)
        or always collide (positive). Zero means no collision group. Non-zero group
        filtering always wins against the mask bits.

        Setting this will not update contacts until the next time 
        step when either parent body is active and awake. Automatically 
        calls refilter.
        """
        return self._group_index

    @group_index.setter
    def group_index(self, group_index):
        self._group_index = group_index
        self._refilter()

    #--- end properties---
    def test_point(self, p):
        """
        Test a point for containment in this fixture.
        @param p a point in world coordinates.
        """
        return self._shape.test_point(self._body._xf, p)

    def ray_cast(self, p1, p2, max_fraction, child_index):
        """
        Cast a ray against a child shape of this fixture's shape.
        Returns: (hit, normal, fraction)
        """
        return self._shape.ray_cast(p1, p2, max_fraction, self._body._xf, child_index)

    def get_aabb(self, child_index):
        """
        Get the fixture's AABB. This AABB may be enlarge and/or stale.
        If you need a more accurate AABB, compute it using the shape and
        the body transform.
        """
        return self._proxies[child_index].aabb

    def _create_proxies(self, broadphase, xf):
        """This supports body activation/deactivation"""
        assert(len(self._proxies)==0)

        self._proxies=[]
        temp_aabb = AABB()
        for i in range(self._shape.child_count):
            aabb = self._shape.compute_aabb(xf, i, temp_aabb)
            # temp_aabb is copied for each fixture proxy
            proxy = FixtureProxy(self, temp_aabb, i)

            proxy.proxy_ref = broadphase.create_proxy(proxy.aabb, proxy)
            self._proxies.append(proxy)

    def _destroy_proxies(self, broadphase):
        """Destroy proxies in the broad-phase"""
        for proxy in self._proxies:
            broadphase.destroy_proxy(proxy.proxy_ref)
            proxy.proxy_ref = None

    def _refilter(self):
        if not self._body:
            return

        # Flag associated contacts for filtering
        for contact in self._body._contacts:
            if self in (contact._fixture_a, contact._fixture_b):
                contact._flag_for_filtering()

        world = self.world
        if not world:
            return

        # Touch each proxy so that new pairs may be created
        broadphase = world.contact_manager.broadphase
        for proxy in self._proxies:
            broadphase.touch_proxy(proxy.proxy_ref)

    def _synchronize(self, broadphase, xf1, xf2):
        aabb1 = AABB()
        aabb2 = AABB()
        for proxy in self._proxies:
            # Compute an AABB that covers the swept shape (may miss some rotation effect).
            self._shape.compute_aabb(xf1, proxy.child_index, aabb1)
            self._shape.compute_aabb(xf2, proxy.child_index, aabb2)

            proxy.aabb.combine_two(aabb1, aabb2)
            displacement = xf2._position - xf1._position

            broadphase.move_proxy(proxy.proxy_ref, proxy.aabb, displacement)

    @property
    def world(self):
        if self._body is not None:
            return self._body._world
        else:
            return None


