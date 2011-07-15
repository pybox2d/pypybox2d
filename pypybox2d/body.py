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

__all__ = ('Body', )
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .common import (LockedError, Vec2, Transform, Sweep, scalar_cross, is_valid_float, property)
from .fixture import Fixture
from .shapes import MassData
from . import shapes

class Body(object):
    """A rigid body."""
    STATIC=0
    DYNAMIC=1
    KINEMATIC=2
    def __init__(self, world=None, position=(0,0), angle=0.0, linear_velocity=(0,0),
                 angular_velocity=0.0, linear_damping=0.0, angular_damping=0.0,
                 allow_sleep=True, awake=True, fixed_rotation=False, bullet=False,
                 type=STATIC, active=True, gravity_scale=1.0, check_args=True,
                 shapes=[], fixtures=[], shape_fixture=None, user_data=None,
                 copy_fixtures=True):

        position = Vec2(*position)
        linear_velocity = Vec2(*linear_velocity)

        if check_args:
            if not position.valid:
                raise ValueError('Invalid position')
            if not linear_velocity.valid:
                raise ValueError('Invalid linear velocity')
            if not is_valid_float(angle):
                raise ValueError('Invalid angle')
            if not is_valid_float(angular_velocity):
                raise ValueError('Invalid angular velocity')
            if not is_valid_float(angular_damping) and angular_damping > 0.0:
                raise ValueError('Invalid angular damping')
            if not is_valid_float(linear_damping) and linear_damping > 0.0:
                raise ValueError('Invalid linear damping')
            if type not in (Body.STATIC, Body.DYNAMIC, Body.KINEMATIC):
                raise ValueError('Invalid body type')

        self._xf = Transform(position = position, angle = angle)
        self._sweep = Sweep()
        self._force = Vec2()
        self._torque = 0.0
        self._I = 0.0 # rotational inertia around the center of mass
        self._invI = 0.0 
        self._sleep_time = 0.0
        self._contacts = []
        self._controllers = []
        self.fixtures = []
        self.joints = []
        self.user_data = user_data

        self._world = world
        self._linear_velocity = linear_velocity
        self._angular_velocity = float(angular_velocity)
        self._linear_damping = float(linear_damping)
        self._angular_damping = float(angular_damping)
        self._gravity_scale = float(gravity_scale)
        self._island_flag = False
        self._island_index = None # throws error if not set
        self._awake = awake
        self._allow_sleep = bool(allow_sleep)
        self._fixed_rotation = bool(fixed_rotation)
        self._bullet = bullet
        self._type = type
        self._active = active

        self._sweep.local_center = Vec2()
        self._sweep.a0 = self._sweep.a = angle
        self._sweep.c0 = self._xf * self._sweep.local_center
        self._sweep.c = copy(self._sweep.c0)

        if self._type==Body.DYNAMIC:
            self._mass=1.0
            self._inv_mass=1.0
        else:
            self._mass=0.0
            self._inv_mass=0.0
        
        if shapes is not None:
            self.create_fixtures(shapes, shape_fixture)

        if fixtures is not None:
            if not isinstance(fixtures, (list, tuple)):
                fixtures = [fixtures]

            for fixture in fixtures:
                if copy_fixtures:
                    self.attach_fixture(copy(fixture))
                else:
                    self.attach_fixture(fixture)

    def __copy__(self):
        new_body = Body(position=self.position, 
                        angle=self.angle,
                        linear_velocity=self._linear_velocity, 
                        angular_velocity=self._angular_velocity, 
                        linear_damping=self._linear_damping,
                        angular_damping=self._angular_damping,
                        allow_sleep=self._allow_sleep,
                        awake=self._awake,
                        fixed_rotation=self._fixed_rotation,
                        bullet=self._bullet,
                        type=self._type,
                        active=self._active,
                        gravity_scale=self._gravity_scale,
                        check_args=False,
                        user_data=self.user_data,
                       )

        new_body.fixtures=[copy(fixture) for fixture in self.fixtures]
        for fixture in new_body.fixtures:
            fixture._body = None
            fixture._attach_to_body(new_body)
        return new_body

    def __repr__(self):
        info = (self.position, 
                self.angle,
                self._linear_velocity, 
                self._angular_velocity, 
                self._linear_damping,
                self._angular_damping,
                self._allow_sleep,
                self._awake,
                self._fixed_rotation,
                self._bullet,
                self._type,
                self._active,
                self._gravity_scale,
                repr(self.user_data),
                self.fixtures)
        return """Body(position=%s, angle=%g, linear_velocity=%s, angular_velocity=%g, 
linear_damping=%s, angular_damping=%g, allow_sleep=%s, awake=%s,
fixed_rotation=%s, bullet=%s, type=%s, active=%s,
gravity_scale=%s, user_data=%s,
fixtures=%s)""" % info

    def create_fixture(self, shape, friction=0.2, restitution=0.0, density=0.0, 
                        category_bits=0x0001, mask_bits=0xFFFF, group_index=0, 
                        sensor=False, user_data=None):
        """
        Creates a fixture and attach it to this body. Use this function if you need
        to set some fixture parameters, like friction. Otherwise you can create the
        fixture directly from a shape.
        If the density is non-zero, this function automatically updates the mass of
        the body.
        Contacts are not created until the next time step.
        @warning This function is locked during callbacks.
        """
        if self._world and self._world.locked:
            raise LockedError

        fixture=Fixture(shape, friction, restitution, density, category_bits, 
                         mask_bits, group_index, sensor, user_data, body=self)

        self.attach_fixture(fixture)
        return fixture

    def attach_fixture(self, fixture):
        """
        Attach an already-created fixture to this body.

        It can exist on another body, but it will first
        be removed from that body.

        For general usage, see create_fixture.
        """
        if fixture in self.fixtures:
            raise ValueError('Fixture is already attached to this body')

        if fixture._body != self and fixture._body is not None:
            fixture._body.destroy_fixture(fixture)
            fixture._body = self
        
        fixture._attach_to_body(self)
        self.fixtures.append(fixture)

        if self._world and self.active:
            fixture._create_proxies(self._broadphase, self._xf)

        # Adjust mass properties if needed.
        if fixture._density > 0.0:
            self.reset_mass_data()

        if self._world:
            # Let the world know we have a new fixture. This will cause new contacts
            # to be created at the beginning of the next time step.
            self._world._new_fixture=True

    def create_circle_fixture(self, radius=0.2, position=(0,0), **kwargs):
        """TODO"""
        shape = shapes.Circle(radius, position)
        return self.create_fixture(shape, **kwargs)

    def create_polygon_fixture(self, vertices=None, box=None, **kwargs):
        """TODO"""
        shape = shapes.Polygon(vertices, box)
        return self.create_fixture(shape, **kwargs)

    def create_edge_fixture(self, v1=None, v2=None, v0=None, v3=None, **kwargs):
        """TODO"""
        shape = shapes.Edge(v1, v2, v0, v3)
        return self.create_fixture(shape, **kwargs)

    def create_loop_fixture(self, vertices, *args, **kwargs):
        """TODO"""
        shape = shapes.Loop(vertices, *args)
        return self.create_fixture(shape, **kwargs)

    def create_fixtures(self, shape_list, fixture=None, friction=0.2, restitution=0.0, 
                        density=0.0, category_bits=0x0001, mask_bits=0xFFFF,
                        group_index=0, sensor=False):
        """TODO"""

        if isinstance(shape_list, shapes.Shape):
            shape_list = [shape_list]

        if fixture is not None:
            friction = fixture.friction
            restitution = fixture.restitution
            density = fixture.density
            category_bits = fixture.category_bits
            mask_bits = fixture.mask_bits
            group_index = fixture.group_index
            sensor = fixture.sensor

        for shape in shape_list:
            self.create_fixture(shape, friction, restitution,
                                density, category_bits, mask_bits, 
                                group_index, sensor)
            
    def destroy_fixture(self, fixture):
        """
        Destroy a fixture. This removes the fixture from the broad-phase and
        destroys all contacts associated with this fixture. This will
        automatically adjust the mass of the body if the body is dynamic and the
        fixture has positive density.
        All fixtures attached to a body are implicitly destroyed when the body is destroyed.
        @param fixture the fixture to be removed.
        @warning This function is locked during callbacks.
        """
        if self._world and self._world.locked:
            raise LockedError

        if fixture._body != self or fixture not in self.fixtures:
            raise ValueError('This fixture does not belong to this body')

        self.fixtures.remove(fixture)

        for contact in list(self._contacts):
            if fixture == contact.fixture_a or fixture == contact.fixture_b:
                # This destroys the contact and removes it from
                # this body's contact list.
                self._world.contact_manager.destroy(contact)

        if self._active:
            fixture._destroy_proxies(self._broadphase)
        
        fixture._body=None
        self.reset_mass_data()

    def _attached_to_world(self, world):
        """The world tells us when the body has been added to it"""
        self._world = world

        active = self.active
        reset_mass_data=False
        if self.fixtures:
            for fixture in self.fixtures:
                if active:
                    fixture._create_proxies(self._broadphase, self._xf)
                
                if fixture._density > 0.0:
                    reset_mass_data=True

            # Let the world know we have a new fixture. This will cause new contacts
            # to be created at the beginning of the next time step.
            world._new_fixture=True

        # Adjust mass properties if needed.
        if reset_mass_data:
            self.reset_mass_data()

    def _detached_from_world(self):
        world = self._world

        # Delete the attached joints
        for joint in list(self.joints):
            if world._joint_destruction_listener:
                world._joint_destruction_listener(joint)

            world.destroy_joint(joint)

        del self.joints[:] # clear the list in place

        # Destroy the attached contacts
        contact_manager = world.contact_manager
        for contact in list(self._contacts):
            contact_manager.destroy(contact)

        del self._contacts[:]

        # And fixtures (also destroying broad-phase proxies)
        broadphase = contact_manager.broadphase
        for fixture in list(self.fixtures):
            if world._fixture_destruction_listener:
                world._fixture_destruction_listener(fixture)

            fixture._destroy_proxies(broadphase)

        broadphase = self._broadphase

        for fixture in self.fixtures:
            fixture._destroy_proxies(broadphase)

        # Remove the controller connections
        for controller in self._controllers:
            controller.remove_body(self)

        self._world = None

    def _attached_to_controller(self, controller):
        """The controller tells us when the body has been added to it"""
        self._controllers.append(controller)

    def _detached_from_controller(self, controller):
        self._controllers.remove(controller)

    @property
    def dynamic(self):
        """Is this a dynamic body?"""
        return self._type == Body.DYNAMIC

    @property
    def static(self):
        """Is this a static body?"""
        return self._type == Body.STATIC

    @property
    def kinematic(self):
        """Is this a kinematic body?"""
        return self._type == Body.KINEMATIC

    @property
    def transform(self):
        """
        The body transform for the body's origin.

        To set it, you may pass in a new Transform() or a tuple of:
            (position, angle)
        Setting the transform breaks any contacts and wakes the other bodies.
        Manipulating a body's transform may cause non-physical behavior.
        @param position the world position of the body's local origin.
        @param angle the world rotation in radians.
        """
        return copy(self._xf)

    @transform.setter
    def transform(self, position_angle):
        if self._world and self._world.locked:
            raise LockedError

        if isinstance(position_angle, Transform):
            transform = copy(position_angle)
        else:
            transform = Transform(position=position_angle[0], angle=position_angle[1])

        self._xf = transform
        self._transform_updated()
    
    @property
    def _broadphase(self):
        try:
            return self._world.contact_manager.broadphase
        except AttributeError:
            return None

    def _transform_updated(self):
        self._sweep.c0 = self._xf * self._sweep.local_center
        self._sweep.c = copy(self._sweep.c0)
        self._sweep.a0 = self._sweep.a = self._xf.angle

        broadphase = self._broadphase
        for fixture in self.fixtures:
            fixture._synchronize(broadphase, self._xf, self._xf)

        self._world.contact_manager.find_new_contacts()

    @property
    def position(self):
        """The world body origin"""
        return Vec2(*self._xf.position)

    @position.setter
    def position(self, pos):
        self._xf.position = pos
        self._transform_updated()

    @property
    def angle(self):
        """The world rotation angle in radians"""
        return self._xf.angle

    @angle.setter
    def angle(self, angle):
        self._xf.angle=angle
        self._transform_updated()

    @property
    def world(self):
        """ The world this body lives in"""
        return self._world

    @property
    def world_center(self):
        """ Get the world position of the center of mass. (copied)"""
        return copy(self._sweep.c)

    @property
    def local_center(self):
        """ Get the local position of the center of mass. (copied)"""
        return copy(self._sweep.local_center)

    @property
    def linear_velocity(self):
        """ The linear velocity of the center of mass. (copied) """
        return copy(self._linear_velocity)
        
    @linear_velocity.setter
    def linear_velocity(self, v):
        if self._type==Body.STATIC:
            return
        v=Vec2(*v)
        if v.dot(v) > 0.0:
            self.awake=True
        self._linear_velocity=v

    @property
    def angular_velocity(self):
        """ The angular velocity in rad/s  """
        return self._angular_velocity
        
    @angular_velocity.setter
    def angular_velocity(self, v):
        if self._type==Body.STATIC:
            return
        if v*v > 0.0:
            self.awake=True
        self._angular_velocity=v

    def apply_force(self, force, point):
        """
        Apply a force at a world point. If the force is not
        applied at the center of mass, it will generate a torque and
        affect the angular velocity. This wakes up the body.
        @param force the world force vector, usually in Newtons (N).
        @param point the world position of the point of application.
        """
        if self._type != Body.DYNAMIC:
            return
        if not self.awake:
            self.awake = True
        
        self._force += force
        self._torque += (point - self._sweep.c).cross(force)

    def apply_force_to_center(self, force):
        """
        Apply a force to the center of mass. This wakes up the body.
        @param force the world force vector, usually in Newtons (N).
        """
        if self._type != Body.DYNAMIC:
            return
        if not self.awake:
            self.awake=True
        
        self._force += force

    def apply_torque(self, torque):
        """
        Apply a torque. This affects the angular velocity
        without affecting the linear velocity of the center of mass.
        This wakes up the body.
        @param torque about the z-axis (out of the screen), usually in N-m.
        """
        if self._type != Body.DYNAMIC:
            return
        if not self.awake:
            self.awake=True
        
        self._torque += torque

    def apply_linear_impulse(self, impulse, point):
        """
        Apply an impulse at a point. This immediately modifies the velocity.
        It also modifies the angular velocity if the point of application
        is not at the center of mass. This wakes up the body.
        @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        @param point the world position of the point of application.
        """
        if self._type != Body.DYNAMIC:
            return
        if not self.awake:
            self.awake=True

        self._linear_velocity += self._inv_mass * Vec2(*impulse)
        self._angular_velocity += self._invI * (Vec2(*point) - self._sweep.c).cross(impulse)

    def apply_angular_impulse(self, impulse):
        """
        Apply an angular impulse.
        @param impulse the angular impulse in units of kg*m*m/s
        """
        if self._type != Body.DYNAMIC:
            return
        if not self.awake:
            self.awake = True

        self._angular_velocity += self._invI * impulse

    @property
    def mass(self):
        """
        Get the total mass of the body.
        @return the mass, usually in kilograms (kg).
        """
        return self._mass

    @property
    def inertia(self):
        """
        Get the rotational inertia of the body about the local origin.
        @return the rotational inertia, usually in kg-m^2.
        """
        return self._I + self._mass * self._sweep.local_center.length_squared

    @property
    def mass_data(self):
        """
        The mass data of the body.

        Setting the mass properties to override the mass properties of the fixtures:
        Note that this changes the center of mass position.
        Note that creating or destroying fixtures can also alter the mass.
        This has no effect if the body isn't dynamic.
        """
        return MassData(mass=self._mass,
                        I=self._I + self._mass * (self._sweep.local_center*self._sweep.local_center),
                        center=self._sweep.local_center)

    @mass_data.setter
    def mass_data(self, md):
        if self._world and self._world.locked:
            raise LockedError
        if self._type != Body.DYNAMIC:
            return
        
        self._inv_mass=0.0
        self._I=0.0
        self._invI=0.0
        self._mass=md.mass
        if self._mass <= 0.0:
            self._mass = 1.0

        self._inv_mass = 1.0 / self._mass
        if md.I > 0.0 and not self._fixed_rotation:
            self._I = md.I - self._mass * (md.center*md.center)
            assert(self._I > 0.0)
            self._invI = 1.0 / self._I

        # Move center of mass.
        old_center = self._sweep.c
        self._sweep.local_center = copy(md.center)
        self._sweep.c0 = self._xf * self._sweep.local_center 
        self._sweep.c = copy(self._sweep.c0)

        # Update center of mass velocity.
        self._linear_velocity += scalar_cross(self.angular_velocity, self._sweep.c - old_center)

    def reset_mass_data(self):
        """
        This resets the mass properties to the sum of the mass properties of the fixtures.
        This normally does not need to be called unless you set mass_data to override
        the mass and you later want to reset the mass.
        """
        # Compute mass data from shapes. Each shape has its own density.
        self._mass = 0.0
        self._inv_mass = 0.0
        self._I = 0.0
        self._invI = 0.0
        self._sweep.local_center = Vec2()

        if self._type in (Body.STATIC, Body.KINEMATIC):
            self._sweep.c = self._xf.position
            self._sweep.c0 = copy(self._sweep.c)
            return

        # Accumulate mass over all fixtures
        center = Vec2()
        for fixture in self.fixtures:
            if fixture._density==0.0:
                continue

            md=fixture.mass_data
            self._mass += md.mass
            center += md.mass * md.center
            self._I += md.I

        # Compute center of mass.
        if self._mass > 0.0:
            self._inv_mass = 1.0 / self._mass
            center *= self._inv_mass
        else:
            # Force all dynamic bodies to have a positive mass.
            self._mass = 1.0
            self._inv_mass = 1.0
       
        if self._I > 0.0 and not self._fixed_rotation:
            # Center the inertia about the center of mass.
            self._I -= self._mass * center.length_squared
            assert(self._I > 0.0)
            self._invI = 1.0 / self._I
        else:
            self._I = 0.0
            self._invI = 0.0

        # Move center of mass.
        old_center = self._sweep.c
        self._sweep.local_center = center
        self._sweep.c0 = self._xf * self._sweep.local_center
        self._sweep.c = copy(self._sweep.c0)

        # Update center of mass velocity.
        self._linear_velocity += scalar_cross(self._angular_velocity, self._sweep.c - old_center)

    def get_world_point(self, local_point):
        """
        Get the world coordinates of a point given the local coordinates.
        @param local_point a point on the body measured relative the the body's origin.
        @return the same point expressed in world coordinates.
        """
        return self._xf * local_point
    
    def get_world_vector(self, local_vector):
        """
        Get the world coordinates of a vector given the local coordinates.
        @param local_vector a vector fixed in the body.
        @return the same vector expressed in world coordinates.
        """
        return self._xf._rotation * local_vector

    def get_local_point(self, world_point):
        """
        Gets a local point relative to the body's origin given a world point.
        @param a point in world coordinates.
        @return the corresponding local point relative to the body's origin.
        """
        return self._xf.mul_t(world_point)
    
    def get_local_vector(self, world_vector):
        """
        Gets a local vector given a world vector.
        @param a vector in world coordinates.
        @return the corresponding local vector.
        """
        return self._xf._rotation.mul_t(world_vector)

    def get_linear_velocity_from_world_point(self, world_point):
        """
        Get the world linear velocity of a world point attached to this body.
        @param a point in world coordinates.
        @return the world velocity of a point.
        """
        return self._linear_velocity + scalar_cross(self._angular_velocity, world_point - self._sweep.c)

    def get_linear_velocity_from_local_point(self, local_point):
        """
        Get the world velocity of a local point.
        @param a point in local coordinates.
        @return the world velocity of a point.
        """
        return self.get_linear_velocity_from_world_point(self.get_world_point(local_point))

    @property
    def linear_damping(self):
        """The linear damping of the body."""
        return self._linear_damping

    @linear_damping.setter
    def linear_damping(self, damping):
        self._linear_damping=float(damping)

    @property
    def angular_damping(self):
        """The angular damping of the body."""
        return self._angular_damping

    @angular_damping.setter
    def angular_damping(self, damping):
        self._angular_damping=float(damping)

    @property
    def gravity_scale(self):
        """The gravity scale of the body."""
        return self._gravity_scale

    @gravity_scale.setter
    def gravity_scale(self, scale):
        self._gravity_scale=float(scale)

    @property
    def type(self):
        """The type of this body. Setting it may alter the mass and velocity."""
        return self._type

    @type.setter
    def type(self, _type):
        if self._type==_type:
            return
        if _type not in (Body.STATIC, Body.DYNAMIC, Body.KINEMATIC):
            raise ValueError('Invalid body type')

        self._type = _type

        self.reset_mass_data()

        if _type == Body.STATIC:
            self._linear_velocity = Vec2()
            self._angular_velocity = 0.0

        self.awake = True
        self._force = Vec2()
        self._torque = 0.0

        # Since the body type changed, we need to flag contacts for filtering.
        for fixture in self.fixtures:
            fixture._refilter()

    @property
    def bullet(self):
        """Is this body treated like a bullet for continuous collision detection?"""
        return self._bullet

    @bullet.setter
    def bullet(self, bullet):
        self._bullet=bullet

    @property
    def allow_sleep(self):
        """
        Is this body allowed to sleep?

        You can disable sleeping on this body. If you disable sleeping, the 
        body will be woken.
        """
        return self._allow_sleep

    @allow_sleep.setter
    def allow_sleep(self, allow_sleep):
        self._allow_sleep=allow_sleep

        if not allow_sleep:
            self.awake=True

    @property
    def awake(self):
        """
        If the body is awake (vs sleeping)
        A sleeping body has very low CPU cost.
        @return true if the body is awake.
        """
        return self._awake

    @awake.setter
    def awake(self, awake):
        if awake:
            if not self._awake:
                self._sleep_time=0.0
        else:
            self._sleep_time=0.0
            self._linear_velocity = Vec2()
            self._angular_velocity = 0.0
            self._force = Vec2()
            self._torque = 0.0

        self._awake=awake

    @property
    def sleeping(self):
        """
        The sleeping state of this body.
        A sleeping body has very low CPU cost.
        @return true if the body is sleeping.
        """
        return not self._awake

    @sleeping.setter
    def sleeping(self, sleeping):
        self.awake = not sleeping

    @property
    def active(self):
        """
        The active state of the body.

        Setting the active state of the body: an inactive body is not
        simulated and cannot be collided with or woken up.
        If you pass a flag of true, all fixtures will be added to the
        broad-phase.
        If you pass a flag of false, all fixtures will be removed from
        the broad-phase and all contacts will be destroyed.
        Fixtures and joints are otherwise unaffected. You may continue
        to create/destroy fixtures and joints on inactive bodies.
        Fixtures on an inactive body are implicitly inactive and will
        not participate in collisions, ray-casts, or queries.
        Joints connected to an inactive body are implicitly inactive.
        An inactive body is still owned by a World object and remains
        in the body list.
        """
        return self._active

    @active.setter
    def active(self, active):
        if self._world and self._world.locked:
            raise LockedError
        
        if self._active == active:
            return

        self._active=active

        broadphase=self._broadphase
        if active:
            # Create all proxies
            for fixture in self.fixtures:
                fixture._create_proxies(broadphase, self._xf)
        else:
            # Destroy all proxies
            for fixture in self.fixtures:
                fixture._destroy_proxies(broadphase)

            # Destroy the attached contacts
            for contact in self._contacts:
                self._world.contact_manager.destroy(contact)

            self._contacts=[]
    
    @property
    def fixed_rotation(self):
        """
        Does this body have fixed rotation?

        Setting this body to have fixed rotation causes the mass
        to be reset.
        """
        return self._fixed_rotation

    @fixed_rotation.setter
    def fixed_rotation(self, fixed_rotation):
        self._fixed_rotation=bool(fixed_rotation)
        self.reset_mass_data()

    @property
    def contact_list(self):
        """
        Get the list of all contacts attached to this body.
        @warning this list changes during the time step and you may
        miss some collisions if you don't use a ContactListener.
        """
        return self._contacts

    def _synchronize_fixtures(self):
        xf1=Transform(angle=self._sweep.a0)
        xf1.position = self._sweep.c0 - (xf1._rotation * self._sweep.local_center)
      
        broadphase=self._broadphase
        for fixture in self.fixtures:
            fixture._synchronize(broadphase, xf1, self._xf)

    def _synchronize_transform(self):
        self._xf = Transform(angle=self._sweep.a)
        self._xf.position = self._sweep.c - (self._xf._rotation * self._sweep.local_center)

    def _should_collide(self, other):
        """
        This is used to prevent connected bodies from colliding.
        It may lie, depending on the collide_connected flag.
        """

        # At least one body should be dynamic.
        if self._type != Body.DYNAMIC and other._type != Body.DYNAMIC:
            return False

        # Does a joint prevent collision?
        for joint in self.joints:
            if (joint.body_a == self and joint.body_b == other) or \
               (joint.body_b == self and joint.body_a == other):

               if not joint._collide_connected:
                   return False

        return True

    def _advance(self, alpha):
        """Advance to the new safe time. This doesn't sync the broad-phase."""
        self._sweep.advance(alpha)
        center = self._sweep.c = copy(self._sweep.c0)
        angle = self._sweep.a = self._sweep.a0
        
        self._xf._rotation.angle = angle
        self._xf.position = center - self._xf._rotation * self._sweep.local_center

