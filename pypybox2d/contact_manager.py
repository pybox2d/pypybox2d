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

__all__ = ('Island', 'ContactManager')
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import (clamp, Vec2, property)
from .broadphase import BroadPhase
from .contact import (Contact, ContactSolver)
from . import settings

MAX_TRANSLATION = settings.MAX_TRANSLATION
MAX_TRANSLATION_SQR = settings.MAX_TRANSLATION_SQR
MAX_ROTATION = settings.MAX_ROTATION
MAX_ROTATION_SQR = settings.MAX_ROTATION_SQR
BAUMGARTE = settings.BAUMGARTE
MAX_FLOAT = settings.MAX_FLOAT
ANGULAR_SLEEP_TOLERANCE_SQR = settings.ANGULAR_SLEEP_TOLERANCE_SQR
LINEAR_SLEEP_TOLERANCE_SQR = settings.LINEAR_SLEEP_TOLERANCE_SQR
TIME_TO_SLEEP = settings.TIME_TO_SLEEP
TOI_BAUMGARTE = settings.TOI_BAUMGARTE
MAX_SUB_STEPS = settings.MAX_SUB_STEPS
MAX_TOI_CONTACTS = settings.MAX_TOI_CONTACTS

class Island(object):
    """This is an internal class."""
    # TODO slots just for debugging
    __slots__ = ['_body_capacity', '_contact_capacity', '_joint_capacity', 'post_solve', 'bodies', 'contacts',
                'joints']
    def __init__(self, body_capacity, contact_capacity, joint_capacity, post_solve):
        self._body_capacity = body_capacity
        self._contact_capacity = contact_capacity
        self._joint_capacity = joint_capacity
        self.post_solve = post_solve
        self.bodies = []
        self.contacts = []
        self.joints = []

    def clear(self):
        del self.bodies[:]
        del self.contacts[:]
        del self.joints[:]
        
    def solve(self, step, gravity, allow_sleep):
        dt = step.dt

        positions = []
        velocities = []
        # Integrate velocities and apply damping. Initialize the body state.
        for body in self.bodies:
            c, a = body._sweep.c, body._sweep.a
            v, w = body._linear_velocity, body._angular_velocity

            # Store positions for continuous collision
            body._sweep.c0 = Vec2(*c)
            body._sweep.a0 = a

            if body.dynamic:
                # Integrate velocities.
                v += dt * (body._gravity_scale * gravity + body._inv_mass * body._force)
                w += dt * body._invI * body._torque

                # Apply damping.
                # ODE: dv/dt + c * v = 0
                # Solution: v(t) = v0 * exp(-c * t)
                # Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                # v2 = exp(-c * dt) * v1
                # Taylor expansion:
                # v2 = (1.0 - c * dt) * v1
                body._linear_velocity *= clamp(1.0 - dt * body._linear_damping, 0.0, 1.0)
                body._angular_velocity *= clamp(1.0 - dt * body._angular_damping, 0.0, 1.0)

            positions.append((c, a))
            velocities.append((v, w))
                
        # Initialize velocity constraints.
        contact_solver=ContactSolver(step, self.contacts, positions, velocities)
        contact_solver.initialize_velocity_constraints()

        if step.warm_starting:
            contact_solver.warm_start()
        
        for joint in self.joints:
            joint._init_velocity_constraints(step, positions, velocities)

        # Solve velocity constraints.
        for i in range(step.vel_iters):
            for joint in self.joints:
                joint._solve_velocity_constraints(step, positions, velocities)
            contact_solver.solve_velocity_constraints()

        # Post-solve (store impulses for warm starting).
        contact_solver.store_impulses()

        body_count = len(self.bodies)

        # Integrate positions.
        for i in range(body_count):
            c, a = positions[i]
            v, w = velocities[i]

            # Check for large velocities.
            translation = dt * v
            if translation.dot(translation) > MAX_TRANSLATION_SQR:
                ratio = MAX_TRANSLATION / translation.length
                v *= ratio

            rotation = dt * w
            if rotation**2 > MAX_ROTATION_SQR:
                ratio = MAX_ROTATION / abs(rotation)
                w *= ratio

            # Integrate
            c += dt * v
            a += dt * w

            positions[i] = (c, a)
            velocities[i] = (v, w)
            
        # Solve position constraints
        for i in range(step.pos_iters):
            contacts_okay = contact_solver.solve_position_constraints()

            joints_okay = True
            for joint in self.joints:
                joint_okay = joint._solve_position_constraints(step, positions, velocities)
                joints_okay = joints_okay and joint_okay

            if contacts_okay and joints_okay:
                # Exit early if the position errors are small.
                break

        # Copy state buffers back to the bodies
        for body, pos, vel in zip(self.bodies, positions, velocities):
            body._sweep.c, body._sweep.a = pos
            body._linear_velocity, body._angular_velocity = vel
            body._synchronize_transform()
        
        self.report(contact_solver.velocity_constraints)

        if allow_sleep:
            min_sleep_time = MAX_FLOAT

            non_static_bodies = [body for body in self.bodies if not body.static]
            for b in non_static_bodies:
                if not body._allow_sleep or \
                        (b._angular_velocity**2) > ANGULAR_SLEEP_TOLERANCE_SQR or \
                        (b._linear_velocity.length_squared) > LINEAR_SLEEP_TOLERANCE_SQR:
                    b._sleep_time = 0.0
                    min_sleep_time = 0.0
                else:
                    b._sleep_time += dt
                    min_sleep_time = min(min_sleep_time, b._sleep_time)

            if min_sleep_time >= TIME_TO_SLEEP:
                for body in self.bodies:
                    b.awake = False

    def solve_toi(self, sub_step, toi_index_a, toi_index_b):
        # Initialize the body state
        positions = [(body._sweep.c, body._sweep.a) for body in self.bodies]
        velocities = [(body._linear_velocity, body._angular_velocity) for body in self.bodies]

        contact_solver=ContactSolver(sub_step, self.contacts, positions, velocities)

        # Solve position constraints
        for i in range(sub_step.pos_iters):
            contacts_okay=contact_solver.solve_toi_position_constraints(toi_index_a, toi_index_b)
            if contacts_okay:
                break
        
        # Leap of faith to new safe state
        body_a, body_b = self.bodies[toi_index_a], self.bodies[toi_index_b]
        body_a._sweep.c0, body_a._sweep.a0 = positions[toi_index_a]
        body_b._sweep.c0, body_b._sweep.a0 = positions[toi_index_b]

        body_a._sweep.c0 = copy(body_a._sweep.c0)
        body_b._sweep.c0 = copy(body_b._sweep.c0)

        # No warm starting is needed for TOI events because warm
        # starting impulses were applied in the discrete solver.
        contact_solver.initialize_velocity_constraints()

        # Solve velocity constraints.
        for i in range(sub_step.vel_iters):
            contact_solver.solve_velocity_constraints()

        # Don't store the TOI contact forces for warm starting because 
        # they can be quite large.

        dt = sub_step.dt
        # Integrate positions.
        for i, (body, pos, vel) in enumerate(zip(self.bodies, positions, velocities)):
            c, a = pos
            v, w = vel

            # Check for large velocities.
            translation = dt * v
            if translation.length_squared > MAX_TRANSLATION_SQR:
                ratio = MAX_TRANSLATION / translation.length
                v *= ratio

            rotation = dt * w
            if rotation**2 > MAX_ROTATION_SQR:
                ratio = MAX_ROTATION / abs(rotation)
                w *= ratio

            # Integrate
            c += dt * v
            a += dt * w

            # Sync bodies
            positions[i] = (c, a)
            velocities[i] = (v, w)
            body._sweep.c = Vec2(*c)
            body._sweep.a = a
            body._linear_velocity = Vec2(*v)
            body._angular_velocity = w

            # Compute new transform
            body._synchronize_transform()

        self.report(contact_solver.velocity_constraints)

    def report(self, constraints):
        if not self.post_solve:
            return

        for contact, vc in zip(self.contacts, constraints):
            impulses=[(point.normal_impulse, point.tangent_impulse) for point in vc.points]

            self.post_solve(contact, impulses)

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
            for contact in body_b._contacts:
                if contact.other_body(body_b) == body_a:
                    fa, fb = contact._fixture_a, contact._fixture_b
                    ia, ib = contact._index_a, contact._index_b

                    if fa==fixture_a and fb==fixture_b and ia==index_a and ib == index_b:
                        # A contact already exists
                        continue

                    if fa==fixture_b and fb==fixture_a and ia==index_b and ib == index_a:
                        # A contact already exists
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

            if body_a.sleeping and body_b.sleeping:
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
