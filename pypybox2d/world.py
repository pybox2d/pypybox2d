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

__all__ = ('TimeStep', 'World')
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import (LockedError, Vec2, property)
from .body import Body
from .contact_manager import (Island, ContactManager)
from . import joints
from . import settings
from . import distance
from . import toi

MAX_TRANSLATION = settings.MAX_TRANSLATION
MAX_TRANSLATION_SQR = settings.MAX_TRANSLATION_SQR
MAX_ROTATION = settings.MAX_ROTATION
MAX_ROTATION_SQR = settings.MAX_ROTATION_SQR
CONTACT_BAUMGARTE = settings.CONTACT_BAUMGARTE
EPSILON = settings.EPSILON
MAX_FLOAT = settings.MAX_FLOAT
ANGULAR_SLEEP_TOLERANCE_SQR = settings.ANGULAR_SLEEP_TOLERANCE_SQR
LINEAR_SLEEP_TOLERANCE_SQR = settings.LINEAR_SLEEP_TOLERANCE_SQR
TIME_TO_SLEEP = settings.TIME_TO_SLEEP
TOI_BAUMGARTE = settings.TOI_BAUMGARTE
MAX_SUB_STEPS = settings.MAX_SUB_STEPS
MAX_TOI_CONTACTS = settings.MAX_TOI_CONTACTS

class TimeStep(object):
    __slots__ = ['dt', 'inv_dt', 'dt_ratio', 'vel_iters', 'pos_iters', 'warm_starting']
    def __init__(self, dt=0.0, inv_dt=0.0, dt_ratio=0.0, vel_iters=0, 
                 pos_iters=0, warm_starting=True):
        self.dt=float(dt)
        self.inv_dt=float(inv_dt)
        self.dt_ratio=float(dt_ratio)
        self.vel_iters=int(vel_iters)
        self.pos_iters=int(pos_iters)
        self.warm_starting=bool(warm_starting)


class World(object):
    # TODO slots just for debugging
    __slots__ = ['_gravity', '_joint_destruction_listener', 
                 '_fixture_destruction_listener', '_contact_filter', 'bodies',
                 'joints', '_warm_starting', '_continuous_physics', 
                 '_sub_stepping', '_allow_sleep', 'contact_manager', 
                 '_step_complete', '_inv_dt0', '_new_fixture', '_locked',
                '_auto_clear_forces']
    def __init__(self, gravity=(0, -10), do_sleep=True, warm_starting=True, 
                 continuous_physics=True, sub_stepping=False, auto_clear_forces=True,
                 joint_destruction_listener=None, fixture_destruction_listener=None,
                 contact_filter=None):
        self._gravity = Vec2(*gravity)

        self.bodies = []
        self.joints = []
        self._warm_starting = warm_starting
        self._continuous_physics = continuous_physics
        self._sub_stepping = sub_stepping
        self._allow_sleep = do_sleep
        self.joint_destruction_listener = joint_destruction_listener 
        self.fixture_destruction_listener = fixture_destruction_listener 
        self._auto_clear_forces = auto_clear_forces

        self.contact_manager = ContactManager()
        self._step_complete = True
        self._inv_dt0 = 0.0
        self._new_fixture = False
        self._locked = False

        if contact_filter is None:
            self.contact_filter = self.default_contact_filter
        else:
            self.contact_filter = contact_filter

    @property
    def gravity(self):
        return Vec2(*self._gravity)

    @gravity.setter
    def gravity(self, gravity):
        self._gravity = Vec2(*gravity)

    @property
    def joint_destruction_listener(self):
        """Callback when a joint is about to be destroyed. 
        
        Arguments: joint
        """
        return self._joint_destruction_listener
    @joint_destruction_listener.setter
    def joint_destruction_listener(self, fcn):
        self._joint_destruction_listener = fcn

    @property
    def fixture_destruction_listener(self):
        """Callback when a fixture is about to be destroyed. 
        
        Arguments: fixture
        """
        return self._fixture_destruction_listener
    @fixture_destruction_listener.setter
    def fixture_destruction_listener(self, fcn):
        self._fixture_destruction_listener = fcn

    @property
    def contact_filter(self):
        """
        Callback that determines whether two fixtures should collide or not.

        Return True if contact calculations should be performed between these two fixtures.
        @warning for performance reasons this is only called when the AABBs begin to overlap.
        """
        return self._contact_filter

    @contact_filter.setter
    def contact_filter(self, fcn):
        self._contact_filter = fcn
        self.contact_manager.contact_filter = fcn

    @property
    def warm_starting(self):
        """Warm starting"""
        return self._warm_starting
    
    @warm_starting.setter
    def warm_starting(self, warm_starting):
        self._warm_starting = bool(warm_starting)

    @property
    def continuous_physics(self):
        """Continuous physics -- for testing"""
        return self._continuous_physics
    
    @continuous_physics.setter
    def continuous_physics(self, continuous_physics):
        self._continuous_physics = bool(continuous_physics)

    @property
    def sub_stepping(self):
        """Sub stepping"""
        return self._sub_stepping
    
    @sub_stepping.setter
    def sub_stepping(self, sub_stepping):
        self._sub_stepping = bool(sub_stepping)

    @property
    def allow_sleep(self):
        """Allow bodies to sleep"""
        return self._allow_sleep
    
    @allow_sleep.setter
    def allow_sleep(self, allow_sleep):
        self._allow_sleep = bool(allow_sleep)

    @property
    def auto_clear_forces(self):
        """Clear forces after each step"""
        return self._auto_clear_forces
    
    @auto_clear_forces.setter
    def auto_clear_forces(self, auto_clear_forces):
        self._auto_clear_forces = bool(auto_clear_forces)

    def add_body(self, body):
        """
        Add a rigid body to the world.  This does not copy your
        body, so you can still refer to it after adding it to the
        world. 
        
        To create another body or duplicate the body, either use
        copy(body) or see World.create_body()

        Warning: This function is locked during callbacks.
        """
        if self.locked:
            raise LockedError('Cannot create a body while simulating')

        self.bodies.append(body)

        if body._world:
            body._world.destroy_body(body)

        body._attached_to_world(self)

        if body.fixtures:
            self._new_fixture = True
        return body
        
    def create_body(self, body=None, **kwargs):
        """
        Creates (or copies) a rigid body in the world.
        Accepts a single body argument or kwargs for Body().

        Warning: This function is locked during callbacks.
        """
        if self.locked:
            raise LockedError('Cannot create a body while simulating')

        if body is not None:
            body=copy(body)
        else:
            body=Body(**kwargs)

        self.add_body(body)
        return body
   
    def create_dynamic_body(self, body=None, **kwargs):
        return self.create_body(body, type=Body.DYNAMIC, **kwargs)

    def create_static_body(self, body=None, **kwargs):
        return self.create_body(body, type=Body.STATIC, **kwargs)

    def create_kinematic_body(self, body=None, **kwargs):
        return self.create_body(body, type=Body.KINEMATIC, **kwargs)

    def destroy_body(self, body):
        """
        Destroy an already created body

        The body itself is still valid after destruction, so you
        are free to re-attach it to this or another world. Contacts
        and joints, however, will be removed as they pertain to 
        the current world only.
        """
        if self.locked:
            raise LockedError('Cannot remove a body while simulating')

        if body not in self.bodies:
            raise ValueError('Body not in world')

        # Delete the attached joints
        for joint in list(body.joints):
            if self._joint_destruction_listener:
                self._joint_destruction_listener(joint)

            self.destroy_joint(joint)

        del body.joints[:] # clear the list in place

        # And contacts
        for contact in list(body._contacts):
            self.contact_manager.destroy(contact)

        del body._contacts[:]

        # And fixtures (also destroying broad-phase proxies)
        broadphase = self.contact_manager.broadphase
        for fixture in list(body.fixtures):
            if self._fixture_destruction_listener:
                self._fixture_destruction_listener(fixture)

            fixture._destroy_proxies(broadphase)

        self.bodies.remove(body)
        body._detached_from_world()

    def add_joint(self, joint, duplicate=False):
        """
        Create a joint

        Warning: This function is locked during callbacks.

        Note: creating a joint doesn't wake the bodies.
        """
        if self.locked:
            raise LockedError('Cannot create a joint while simulating')

        if duplicate:
            # Duplicate the joint
            joint = copy(joint) 

        # And add it to the list
        self.joints.append(joint)
        
        if joint._body_a is not None:
            joint._body_a.joints.append(joint)
        if joint._body_b is not None:
            joint._body_b.joints.append(joint)

            # If the joint prevents collisions, then flag any contacts for filtering.
            if not joint._collide_connected:
                for contact in joint.body_b._contacts:
                    if contact.other_body(joint.body_b)==joint.body_a:
                        #Flag the contact for filtering at the next time step (where either
                        # body is awake)
                        contact._flag_for_filtering()
    
        return joint

    def destroy_joint(self, joint):
        """Destroy an already created joint"""
        if self.locked:
            raise LockedError('Cannot destroy a joint while simulating')
        elif joint not in self.joints:
            raise ValueError('Joint not in world')

        self.joints.remove(joint)

        for body in joint.bodies:
            if body is not None:
                body.awake = True
                body.joints.remove(joint)

        # If the joint prevents collisions, then flag any contacts for filtering.
        if not joint.collide_connected:
            for contact in joint.body_b._contacts:
                if contact.other_body(joint.body_b)==joint.body_a:
                    # Flag the contact for filtering at the next time step (where either
                    # body is awake)
                    contact._flag_for_filtering()

    def step(self, dt, vel_iters, pos_iters):
        """
        Take a time step. This performs collision detection, integration,
        and constraint solution.
        @param dt the amount of time to simulate, this should not vary between calls.
        @param velocityIterations for the velocity constraint solver.
        @param positionIterations for the position constraint solver.
        """

        if self._new_fixture:
            # If new fixtures were added, we need to find the new contacts.
            self.contact_manager.find_new_contacts()
            self._new_fixture = False
       
        self._locked=True

        if dt > 0.0:
            inv_dt = 1.0 / dt
        else:
            inv_dt = 0.0

        dt_ratio = self._inv_dt0 * dt

        step = TimeStep(dt = dt, 
                         inv_dt = inv_dt,
                         dt_ratio = dt_ratio,
                         vel_iters = vel_iters,
                         pos_iters = pos_iters,
                         warm_starting = self._warm_starting)

        # Update contacts. This is where some contacts are destroyed.
        self.contact_manager.collide()

        # Integrate velocities, solve velocity constraints, and integrate positions.
        if self._step_complete and step.dt > 0.0:
            self._solve(step)

        # Handle TOI events
        if self._continuous_physics and step.dt > 0.0:
            self.solve_toi(step)

        if step.dt > 0.0:
            self._inv_dt0=step.inv_dt

        if self._auto_clear_forces:
            self.clear_forces()

        self._locked=False

    def clear_forces(self):
        """
        Manually clear the force buffer on all bodies. By default, forces are cleared automatically
        after each call to Step. The default behavior is modified by calling SetAutoClearForces.
        The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
        a fixed sized time step under a variable frame-rate.
        When you perform sub-stepping you will disable auto clearing of forces and instead call
        ClearForces after all sub-steps are complete in one pass of your game loop.
        @see SetAutoClearForces
        """

        for body in self.bodies:
            body._force.zero()
            body._torque=0.0

    def query_aabb(self, aabb, callback=None):
        """
        Query the world for all fixtures that potentially overlap the
        provided AABB.
        Note: generator: yields each fixture
        @param aabb the query box.
        """
        broadphase = self._broadphase
        if callback is not None:
            for result in broadphase.query(aabb):
                if not callback(result.user_data.fixture):
                    break
        else:
            for result in broadphase.query(aabb):
                yield result.user_data.fixture

    def ray_cast(self, point1, point2, callback=None):
        """
        Ray-cast the world for all fixtures in the path of the ray. Your callback
        controls whether you get the closest point, any point, or n-points.
        The ray-cast ignores shapes that contain the starting point.
        Note: generator, alternatively uses a callback.

        Yields a list: [fixture, point, normal, fraction]
            fixture: the fixture hit by the ray
            point: the point of initial intersection
            normal: the normal vector at the point of intersection
            fraction: 
                Used as a return value when used as a generator.
                Modify this value in place as follows for the desired effect:

            for ret in world.ray_cast(p1, p2):
                fixture, point, normal, fraction = ret

                # do something

                ret[-1] = -1: ignore this fixture and continue
                ret[-1] = 0: terminate the ray cast
                ret[-1] = fraction: clip the ray to this point
                ret[-1] = 1: don't clip the ray and continue

        If a callback is specified, the parameters are the same:

            def callback(fixture, point, normal, fraction):
                # do something
                return -1 # ignore this fixture and continue

        @param point1 the ray starting point
        @param point2 the ray ending point
        @param callback (optional) callback to be used instead of a generator
        """
        broadphase = self._broadphase
        point1 = Vec2(*point1)
        point2 = Vec2(*point2)
        for pair in broadphase.ray_cast(point1, point2, 1.0):
            max_fraction, proxy = pair
            fixture_proxy = proxy.user_data
            fixture = fixture_proxy.fixture
            index = fixture_proxy.child_index
            hit, normal, fraction = fixture.ray_cast(point1, point2, 1.0, index)
            if hit:
                point = (1.0 - fraction) * point1 + fraction * point2
                ret_list = [fixture, point, normal, fraction]
                if callback is not None:
                    pair[0] = callback(*ret_list)
                else:
                    yield ret_list
                    pair[0] = ret_list[-1] # return the info to the tree ray cast
            else:
                pair[0] = 1.0 # return the info to the tree ray cast

    @property
    def _broadphase(self):
        return self.contact_manager.broadphase

    @property
    def warm_starting(self):
        """Use warm starting flag"""
        return self._warm_starting
    @warm_starting.setter
    def warm_starting(self, warm_starting):
        self._warm_starting = warm_starting

    @property
    def auto_clear_forces(self):
        """Auto clear forces after a step"""
        return self._auto_clear_forces
    @auto_clear_forces.setter
    def auto_clear_forces(self, auto_clear_forces):
        self._auto_clear_forces = auto_clear_forces

    @property
    def continuous_physics(self):
        """Continuous physics? (TOI)"""
        return self._continuous_physics
    @continuous_physics.setter
    def continuous_physics(self, continuous_physics):
        self._continuous_physics = continuous_physics

    @property
    def locked(self):
        """Is the world locked? (in the middle of a time step)."""
        return self._locked

    def _solve(self, step):
        """Find islands, integrate and solve constraints, solve position constraints"""
        # Size the island for the worst case
        island=Island(len(self.bodies), len(self.contact_manager.contacts), len(self.joints), self.contact_manager.post_solve)

        # Clear all the island flags.
        for body in self.bodies:
            body._island_flag=False

        for contact in self.contact_manager.contacts:
            contact._island_flag=False

        for joint in self.joints:
            joint._island_flag=False

        # Build and simulate all awake islands.
        stack=[]
        for seed in self.bodies:
            if seed._island_flag:
                continue
            if not seed.awake or not seed.active:
                continue

            # The seed can be dynamic or kinematic.
            if seed._type == Body.STATIC:
                continue

            # Reset island and stack.
            island.clear()
            stack.append(seed)
            seed._island_flag=True

            # Perform a depth first search (DFS) on the constraint graph.
            while stack:
                # Grab the next body off the stack and add it to the island.
                body = stack.pop()
                assert(body.active)
                island.bodies.append(body)

                # Make sure the body is awake.
                body.awake = True

                # To keep islands as small as possible, we don't
                # propagate islands across static bodies.
                if body._type == Body.STATIC:
                    continue

                # Search all contacts connected to this body.
                contacts = [contact for contact in body._contacts
                                if not contact._island_flag # Not already added
                                    and contact._enabled_flag # Only solid contacts
                                    and contact._touching_flag  # That are touching
                                    and not contact._fixture_a.sensor # And skip sensors.
                                    and not contact._fixture_b.sensor]

                for contact in contacts:
                    island.contacts.append(contact)
                    contact._island_flag=True

                    other=contact.other_body(body)

                    # Was the other body already added to this island?
                    if other._island_flag:
                        continue

                    stack.append(other)
                    other._island_flag=True

                # Search all joints connect to this body.
                for joint in body.joints:
                    if joint._island_flag:
                        continue

                    other=joint.other_body(body)

                    # Don't simulate joints connected to inactive bodies.
                    if other is None:
                        # Joints that work on one body only (e.g., MouseJoint)
                        island.joints.append(joint)
                        joint._island_flag=True
                        continue

                    elif not other.active:
                        continue

                    island.joints.append(joint)
                    joint._island_flag=True

                    if other._island_flag:
                        continue

                    stack.append(other)
                    other._island_flag=True

            island.solve(step, self._gravity, self._allow_sleep)

            # Post solve cleanup.
            for body in island.bodies:
                # Allow static bodies to participate in other islands.
                if body._type == Body.STATIC:
                    body._island_flag = False

        # Synchronize fixtures, check for out of range bodies.
        for body in self.bodies:
            # If a body was not in an island then it did not move.
            if not body._island_flag or body._type == Body.STATIC:
                continue

            # Update fixtures (for broad-phase).
            body._synchronize_fixtures()

        # Look for new contacts.
        self.contact_manager.find_new_contacts()

    def solve_toi(self, step):
        """Find TOI contacts and solve them."""
        island=Island(2 * MAX_TOI_CONTACTS, MAX_TOI_CONTACTS, 0, self.contact_manager.post_solve)

        listener = self.contact_manager.contact_listener
        if self._step_complete:
            for b in self.bodies:
                b._island_flag = False
                b._sweep.alpha0 = 0.0

            for c in self.contact_manager.contacts:
                # Invalidate TOI
                c._toi_flag = False
                c._island_flag = False
                c._toi_count = 0
                c._toi = 1.0

        # Find TOI events and solve them.
        while True:
            # Find the first TOI.
            min_contact = None
            min_alpha = 1.0

            contacts = [contact for contact in self.contact_manager.contacts
                            if contact.enabled 
                                and contact._toi_count <= MAX_SUB_STEPS]

            # For all enabled contacts (that haven't been sub-stepped
            # excessively)
            for c in contacts:
                alpha = 1.0
                if c._toi_flag:
                    # This contact has a valid cached TOI.
                    alpha = c._toi
                else:
                    fixture_a, fixture_b = c._fixture_a, c._fixture_b

                    # Is there a sensor?
                    if fixture_a.sensor or fixture_b.sensor:
                        continue
                    
                    body_a, body_b = fixture_a.body, fixture_b.body
                    type_a, type_b = body_a._type, body_b._type

                    assert(type_a == Body.DYNAMIC or type_b == Body.DYNAMIC)

                    awake_a = body_a.awake and type_a != Body.STATIC
                    awake_b = body_b.awake and type_b != Body.STATIC

                    # Is at least one body awake?
                    if not awake_a and not awake_b:
                        continue

                    collide_a = body_a.bullet or type_a != Body.DYNAMIC
                    collide_b = body_b.bullet or type_b != Body.DYNAMIC

                    # Are these two non-bullet dynamic bodies?
                    if not collide_a and not collide_b:
                        continue

                    # Compute the TOI for this contact.
                    # Put the sweeps onto the same time interval.
                    alpha0 = body_a._sweep.alpha0
                    if body_a._sweep.alpha0 < body_b._sweep.alpha0:
                        alpha0 = body_b._sweep.alpha0
                        body_a._sweep.advance(alpha0)
                    elif body_b._sweep.alpha0 < body_a._sweep.alpha0:
                        body_b._sweep.advance(alpha0)

                    assert(alpha0 < 1.0)

                    index_a = c._index_a
                    index_b = c._index_b

                    # Compute the time of impact in interval [0, minTOI]
                    proxy_a=distance.DistanceProxy(fixture_a._shape, index_a)
                    proxy_b=distance.DistanceProxy(fixture_b._shape, index_b)

                    state, beta=toi.time_of_impact(proxy_a, proxy_b, body_a._sweep, body_b._sweep, 1.0)

                    # Beta is the fraction of the remaining portion of the .
                    if state == toi.TOUCHING:
                        alpha = min(alpha0 + (1.0 - alpha0) * beta, 1.0)
                    else:
                        alpha = 1.0

                    c._toi = alpha
                    c._toi_flag = True

                if alpha < min_alpha:
                    # This is the minimum TOI found so far.
                    min_contact = c
                    min_alpha = alpha

            if min_contact is None or (1.0 - (10.0 * EPSILON)) < min_alpha:
                # No more TOI events. Done!
                self._step_complete = True
                break

            # Advance the bodies to the TOI.
            fixture_a = min_contact._fixture_a
            fixture_b = min_contact._fixture_b
            body_a = fixture_a._body
            body_b = fixture_b._body

            backup_a = copy(body_a._sweep)
            backup_b = copy(body_b._sweep)

            body_a._advance(min_alpha)
            body_b._advance(min_alpha)

            # The TOI contact likely has some new contact points.
            min_contact.update(*listener)
            min_contact._toi_flag = False
            min_contact._toi_count += 1

            # Is the contact solid?
            if not min_contact.enabled or not min_contact.touching:
                # Restore the sweeps.
                min_contact.enabled = False
                body_a._sweep = backup_a
                body_b._sweep = backup_b
                body_a._synchronize_transform()
                body_b._synchronize_transform()
                continue

            body_a.awake=True
            body_b.awake=True

            # Build the island
            island.clear()
            island.bodies.append(body_a)
            island.bodies.append(body_b)
            island.contacts.append(min_contact)

            body_a._island_flag = True
            body_b._island_flag = True
            min_contact._island_flag = True

            # Get contacts on bodyA and bodyB.
            bodies = [body for body in (body_a, body_b) 
                            if body._type == Body.DYNAMIC]

            for body in bodies:
                for contact in body._contacts:
                    if len(island.bodies) == island._body_capacity:
                        print('island body capacity')
                        break
                    if len(island.contacts) == island._contact_capacity:
                        print('island contact capacity')
                        break

                    # Has this contact already been added to the island?
                    if contact._island_flag:
                        continue

                    # Only add static, kinematic, or bullet bodies.
                    other=contact.other_body(body)
                    if other._type == Body.DYNAMIC and not body.bullet and not other.bullet:
                        continue

                    # Skip sensors.
                    if contact._fixture_a.sensor or contact._fixture_b.sensor:
                        continue

                    # Tentatively advance the body to the TOI.
                    backup = copy(other._sweep)
                    if not other._island_flag:
                        other._advance(min_alpha)

                    # Update the contact points
                    contact.update(*listener)

                    # Was the contact disabled by the user?
                    # -- Or are there contact points?
                    if not contact.enabled or not contact.touching:
                        other._sweep = backup
                        other._synchronize_transform()
                        continue

                    # Add the contact to the island
                    contact._island_flag = True
                    island.contacts.append(contact)

                    # Has the other body already been added to the island?
                    if other._island_flag:
                        continue
                    
                    # Add the other body to the island.
                    other._island_flag = True

                    if other._type != Body.STATIC:
                        other.awake=True

                    island.bodies.append(other)

            sub_dt = (1.0 - min_alpha) * step.dt
            sub_step=TimeStep(dt = sub_dt,
                                inv_dt = 1.0 / sub_dt,
                                dt_ratio = 1.0,
                                pos_iters = 20,
                                vel_iters = step.vel_iters,
                                warm_starting = False)

            island.solve_toi(sub_step, body_a, body_b)

            # Reset island flags and synchronize broad-phase proxies.
            for body in island.bodies:
                body._island_flag = False

                if body._type != Body.DYNAMIC:
                    continue

                body._synchronize_fixtures()

                # Invalidate all contact TOIs on this displaced body.
                for contact in body._contacts:
                    contact._toi_flag=False
                    contact._island_flag=False

            # Commit fixture proxy movements to the broad-phase so that new contacts are created.
            # Also, some contacts can be destroyed.
            self.contact_manager.find_new_contacts()

            if self._sub_stepping:
                self._step_complete = False
                break

    def create_distance_joint(self, *args, **kwargs):
        """
        Create a distance joint between two bodies.

        For more information, see the joints.DistanceJoint class.
        """
        joint = joints.DistanceJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_weld_joint(self, *args, **kwargs):
        """
        Create a weld joint between two bodies.

        For more information, see the joints.WeldJoint class.
        """
        joint = joints.WeldJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_friction_joint(self, *args, **kwargs):
        """
        Create a friction joint between two bodies.

        For more information, see the joints.FrictionJoint class.
        """
        joint = joints.FrictionJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_gear_joint(self, *args, **kwargs):
        """
        Create a gear joint between two joints.

        For more information, see the joints.GearJoint class.
        """
        joint = joints.GearJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_mouse_joint(self, *args, **kwargs):
        """
        Create a mouse joint on a body.

        For more information, see the joints.MouseJoint class.
        """
        joint = joints.MouseJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_prismatic_joint(self, *args, **kwargs):
        """
        Create a prismatic joint between two bodies.

        For more information, see the joints.PrismaticJoint class.
        """
        joint = joints.PrismaticJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_pulley_joint(self, *args, **kwargs):
        """
        Create a pulley joint between two bodies.

        For more information, see the joints.PulleyJoint class.
        """
        joint = joints.PulleyJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_revolute_joint(self, *args, **kwargs):
        """
        Create a revolute joint between two bodies.

        For more information, see the joints.RevoluteJoint class.
        """
        joint = joints.RevoluteJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_rope_joint(self, *args, **kwargs):
        """
        Create a rope joint between two bodies.

        For more information, see the joints.RopeJoint class.
        """
        joint = joints.RopeJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    def create_wheel_joint(self, *args, **kwargs):
        """
        Create a wheel joint between two bodies.

        For more information, see the joints.WheelJoint class.
        """
        joint = joints.WheelJoint(*args, **kwargs)
        self.add_joint(joint)
        return joint

    @staticmethod
    def default_contact_filter(filter_a, filter_b):
        """
        Return True if contact calculations should be performed between these 
        two shapes.  If you implement your own collision filter you may want 
        to build from this implementation.
        """
        if (filter_a._group_index == filter_b._group_index and 
            filter_a._group_index != 0):
            return filter_a._group_index > 0
        else:
            return ((filter_a._mask_bits & filter_b._category_bits) != 0 and 
                    (filter_a._category_bits & filter_b._mask_bits) != 0)
