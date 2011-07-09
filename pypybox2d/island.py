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

__all__ = ('Island', )
__version__ = "$Revision: 349 $"
__date__ = "$Date: 2011-07-08 19:40:46 -0400 (Fri, 08 Jul 2011) $"
# $Source$

from copy import copy
from .common import (clamp, Vec2, property)
from .contact import (ContactSolver, )
from . import settings

MAX_TRANSLATION = settings.MAX_TRANSLATION
MAX_TRANSLATION_SQR = settings.MAX_TRANSLATION_SQR
MAX_ROTATION = settings.MAX_ROTATION
MAX_ROTATION_SQR = settings.MAX_ROTATION_SQR
BAUMGARTE = settings.BAUMGARTE
TOI_BAUMGARTE = settings.TOI_BAUMGARTE
MAX_FLOAT = settings.MAX_FLOAT
ANGULAR_SLEEP_TOLERANCE_SQR = settings.ANGULAR_SLEEP_TOLERANCE_SQR
LINEAR_SLEEP_TOLERANCE_SQR = settings.LINEAR_SLEEP_TOLERANCE_SQR
TIME_TO_SLEEP = settings.TIME_TO_SLEEP
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
        position_solved = False
        for i in range(step.pos_iters):
            contacts_okay = contact_solver.solve_position_constraints()

            joints_okay = True
            for joint in self.joints:
                joint_okay = joint._solve_position_constraints(step, positions, velocities)
                joints_okay = joints_okay and joint_okay

            if contacts_okay and joints_okay:
                # Exit early if the position errors are small.
                position_solved = True
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

            if min_sleep_time >= TIME_TO_SLEEP and position_solved:
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


# -- NOTES --
"""
[Notes from the original Box2D source code]

Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.

--

Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.

--

2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
"""
