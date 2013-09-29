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

__all__ = ('ContactManager')
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from .common import property
from .broadphase import BroadPhase
from .contact import (Contact, )

class ContactManager(object):
    __slots__ = ['broadphase', 'contacts', 'contact_filter', 
                 '_begin_contact', '_end_contact', '_pre_solve', '_post_solve']
    def __init__(self):
        self.broadphase=BroadPhase()
        self.contacts=[]
        self.contact_filter=None

        # Callbacks
        self._begin_contact = None
        self._end_contact = None
        self._pre_solve = None
        self._post_solve = None
   
    @property
    def begin_contact(self):
        """Callback when two fixtures begin to touch. Arguments: contact"""
        return self._begin_contact
    @begin_contact.setter
    def begin_contact(self, fcn):
        self._begin_contact = fcn

    @property
    def end_contact(self):
        """Callback when two fixtures cease to touch. Arguments: contact"""
        return self._end_contact
    @end_contact.setter
    def end_contact(self, fcn):
        self._end_contact = fcn

    @property
    def pre_solve(self):
        """Callback before solving. Arguments: contact, old_manifold
        
        This is called after a contact is updated. This allows you to inspect a
        contact before it goes to the solver. If you are careful, you can modify the
        contact manifold (e.g. disable contact).
        A copy of the old manifold is provided so that you can detect changes.
        Note: this is called only for awake bodies.
        Note: this is called even when the number of contact points is zero.
        Note: this is not called for sensors.
        Note: if you set the number of contact points to zero, you will not
        get an EndContact callback. However, you may get a BeginContact callback
        the next step.
        """
        return self._pre_solve
    @pre_solve.setter
    def pre_solve(self, fcn):
        self._pre_solve = fcn

    @property
    def post_solve(self):
        """Callback after solving. Arguments: contact, impulses

        This lets you inspect a contact after the solver is finished. This is useful
        for inspecting impulses.
        Note: the contact manifold does not include time of impact impulses, which can be
        arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
        in a separate data structure.
        Note: this is only called for contacts that are touching, solid, and awake.
        """
        return self._post_solve
    @post_solve.setter
    def post_solve(self, fcn):
        self._post_solve = (fcn)

    @property
    def contact_listener(self):
        return [self._begin_contact, self._end_contact, self._pre_solve, self._post_solve]

    def find_new_contacts(self):
        for proxy_a, proxy_b in self.broadphase.update_pairs():
            assert(proxy_a and proxy_b)

            fixture_a = proxy_a.fixture
            fixture_b = proxy_b.fixture

            body_a = fixture_a._body
            body_b = fixture_b._body

            # If the fixtures are on the same body, ignore it
            if body_a == body_b:
                continue

            index_a = proxy_a.child_index
            index_b = proxy_b.child_index

            # Does a contact already exist?
            exists = False

            for contact in body_b._contacts:
                if contact.other_body(body_b) == body_a:
                    fa, fb = contact._fixture_a, contact._fixture_b
                    ia, ib = contact._index_a, contact._index_b

                    if fa == fixture_a and fb == fixture_b and ia==index_a and ib == index_b:
                        # A contact already exists
                        exists = True
                        break

                    elif fa == fixture_b and fb == fixture_a and ia==index_b and ib == index_a:
                        # A contact already exists
                        exists = True
                        break

            if exists:
                continue

            # Does a joint override collision? Is at least one body dynamic?
            if not body_b._should_collide(body_a):
                continue
            
            # Check user filtering.
            if self.contact_filter is not None and not self.contact_filter(fixture_a, fixture_b):
                continue
            
            # Create a contact through the factory.
            c = Contact._create(fixture_a, index_a, fixture_b, index_b)
            if not c:
                # Collisions between these types not defined/registered! This is not good.
                pass
            else:
                self.contacts.append(c)

                # Connect to bodies
                body_a._contacts.append(c)
                body_b._contacts.append(c)

    def destroy(self, contact):
        fixture_a = contact._fixture_a
        fixture_b = contact._fixture_b

        body_a = fixture_a.body
        body_b = fixture_b.body

        if contact.touching and self.end_contact:
            self.end_contact(contact)

        self.contacts.remove(contact)
        body_a._contacts.remove(contact)
        body_b._contacts.remove(contact)

    def collide(self):
        """
        This is the top level collision call for the time step. Here
        all the narrow phase collision is processed for the world contact list.
        """
        to_remove = []
        listener = self.contact_listener
        for contact in self.contacts:
            fixture_a = contact._fixture_a
            fixture_b = contact._fixture_b

            index_a = contact._index_a
            index_b = contact._index_b

            body_a = fixture_a._body
            body_b = fixture_b._body

            assert(body_a.dynamic or body_b.dynamic)

            active_a = body_a.awake and not body_a.static
            active_b = body_b.awake and not body_b.static
            if not active_a and not active_b:
                # At least one body must be awake and it must be dynamic or kinematic.
                continue
        
            # Is this contact flagged for filtering?
            if contact._filter_flag:
                # Should these bodies collide?
                if not body_b._should_collide(body_a) or (self.contact_filter is not None and
                        not self.contact_filter(fixture_a, fixture_b)):
                    to_remove.append(contact)
                    continue
                
                # Clear the filtering flag.
                contact._filter_flag = False
            
            pid_a = fixture_a._proxies[index_a].proxy_ref
            pid_b = fixture_b._proxies[index_b].proxy_ref

            overlap = self.broadphase.test_overlap(pid_a, pid_b)

            if not overlap:
                # Here we destroy contacts that cease to overlap in the broad-phase.
                to_remove.append(contact)
                continue
            
            contact.update(*listener)

        for remove in to_remove:
            self.destroy(remove)
