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

__all__ = ('Contact', 'ContactSolver')

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from . import shapes
from . import distance
from . import settings
from . import collision
from .common import (Vec2, Mat22, scalar_cross, clamp, Transform, property)
from .contact_util import (mix_friction, mix_restitution, 
                           Manifold, WorldManifold, 
                           ContactRegister, ContactVelocityConstraint,
                           ContactPositionConstraint, VelocityConstraintPoint)

EPSILON = settings.EPSILON
EPSILON_SQR = settings.EPSILON_SQR
VELOCITY_THRESHOLD = settings.VELOCITY_THRESHOLD
DEBUG_SOLVER = settings.DEBUG_SOLVER
LINEAR_SLOP = settings.LINEAR_SLOP
MAX_LINEAR_CORRECTION = settings.MAX_LINEAR_CORRECTION
TOI_BAUMGARTE = settings.TOI_BAUMGARTE
BAUMGARTE = settings.BAUMGARTE

class Contact(object):
    """
    The class manages contact between two shapes. A contact exists for each overlapping
    AABB in the broad-phase (except if filtered). Therefore a contact object may exist
    that has no contact points.
    """
    _initialized = False
    _registers = None
    def __init__(self, evaluate_fcn, fixture_a=None, fixture_b=None, index_a=0, index_b=0):
        self.evaluate = evaluate_fcn 
        self._fixture_a = fixture_a
        self._fixture_b = fixture_b
        self._index_a = index_a
        self._index_b = index_b
        
        self._toi = 0.0
        self._toi_count = 0
        self._friction = mix_friction(fixture_a._friction, fixture_b._friction)
        self._restitution = mix_restitution(fixture_a._restitution, fixture_b._restitution)
        self._manifold = Manifold()
        
        # Used when crawling contact graph when forming islands.
        self._island_flag=False
        # Set when the shapes are touching.
        self._touching_flag=False
        # This contact can be disabled (by user)
        self._enabled_flag=False
        # This contact needs filtering because a fixture filter was changed.
        self._filter_flag=False
        # This bullet contact had a TOI event
        self._bullet_hit_flag=False
        # This contact has a valid TOI in m_toi
        self._toi_flag=False

    @property
    def manifold(self):
        """
        Get the contact manifold. Do not modify the manifold unless you understand the
        internals of Box2D.
        """
        return self._manifold

    def other_body(self, body):
        """
        Given one body, return the other related to this contact.
        """
        if body == self._fixture_a._body:
            return self._fixture_b._body
        else:
            return self._fixture_a._body

    @property
    def world_manifold(self):
        """ Get the world manifold."""
        return WorldManifold(self._manifold, 
                             self.body_a._xf, self.shape_a.radius, 
                             self.body_b._xf, self.shape_b.radius)

    @property
    def touching(self):
        """ Is this contact touching?"""
        return self._touching_flag
   
    @property
    def enabled(self):
        """
        Has this contact been disabled?

        Enable/disable this contact. This can be used inside the pre-solve
        contact listener. The contact is only disabled for the current
        time step (or sub-step in continuous collisions).
        """
        return self._enabled_flag

    @enabled.setter
    def enabled(self, enabled):
        self._enabled_flag = enabled

    @property
    def body_a(self):
        """ Get body A in this contact."""
        return self._fixture_a.body

    @property
    def body_b(self):
        """ Get body B in this contact."""
        return self._fixture_b.body

    @property
    def shape_a(self):
        """ Get shape A in this contact."""
        return self._fixture_a._shape

    @property
    def shape_b(self):
        """ Get shape B in this contact."""
        return self._fixture_b._shape

    @property
    def fixture_a(self):
        """ Get fixture A in this contact."""
        return self._fixture_a

    @property
    def child_index_a(self):
        """ Get the child primitive index for fixture A."""
        return self._index_a

    @property
    def fixture_b(self):
        """ Get fixture B in this contact."""
        return self._fixture_b

    @property
    def child_index_b(self):
        """ Get the child primitive index for fixture B."""
        return self._index_b

    @property
    def friction(self):
        """ 
        Friction for the contact.

        override the default friction mixture. You can call this in contact_listener.pre_solve.
        This value persists until set or reset.
        """
        return self._friction

    @friction.setter
    def friction(self, friction):
        self._friction = friction

    def reset_friction(self):
        """ Reset the friction mixture to the default value."""
        return mix_friction(self._fixture_a._friction, self._fixture_b._friction)

    @property
    def restitution(self):
        """ 
        Restitution for the contact.

        override the default restitution mixture. You can call this in contact_listener.pre_solve.
        The value persists until you set or reset.
        """
        return self._restitution

    @restitution.setter
    def restitution(self, restitution):
        self._restitution = restitution

    def reset_restitution(self):
        """ Reset the restitution mixture to the default value."""
        return mix_restitution(self._fixture_a._restitution, self._fixture_b._restitution)

    def evaluate(self, manifold, xf_a, xf_b):
        """ Evaluate this contact with your own manifold and transforms."""
        raise NotImplementedError # implemented in subclasses

    def _flag_for_filtering(self):
        self._filter_flag = True

    def update(self, begin_contact, end_contact, pre_solve, post_solve):
        """
        Update the contact manifold and touching status.
        Note: do not assume the fixture AABBs are overlapping or are valid.
        """
        old_manifold = copy(self._manifold)

        # Re-enable this contact.
        self._enabled_flag = True

        touching = False
        was_touching = self._touching_flag

        sensor_a = self._fixture_a.sensor
        sensor_b = self._fixture_b.sensor
        sensor = sensor_a or sensor_b

        body_a = self._fixture_a._body
        body_b = self._fixture_b._body
        xf_a = body_a._xf
        xf_b = body_b._xf

        # Is this contact a sensor?
        if sensor:
            shape_a = self._fixture_a._shape
            shape_b = self._fixture_b._shape
            touching = distance.test_overlap(shape_a, self._index_a, shape_b, self._index_b, xf_a, xf_b)

            # Sensors don't generate manifolds.
            self._manifold.point_count = 0
        else:
            self.evaluate(self, self._manifold, xf_a, xf_b) 
            # ^^ note: self twice is correct, because of how this method was dynamically set
            touching = (self._manifold.point_count > 0)

            # Match old contact ids to new contact ids and copy the
            # stored impulses to warm start the solver.
            old_impulses = dict((mp.id, (mp.normal_impulse, mp.tangent_impulse))
                                    for mp in old_manifold.used_points)
            for mp2 in self._manifold.used_points:
                id2 = mp2.id
                if id2 in old_impulses:
                    mp2.normal_impulse, mp2.tangent_impulse = old_impulses[id2]
                else:
                    mp2.normal_impulse = 0.0
                    mp2.tangent_impulse = 0.0

            if touching != was_touching:
                body_a.awake = True
                body_b.awake = True

        self._touching_flag = touching

        if begin_contact is not None and (not was_touching and touching):
            begin_contact(self)
        if end_contact is not None and (was_touching and not touching):
            end_contact(self)
        if pre_solve is not None and (not sensor and touching):
            pre_solve(self, old_manifold)

    @staticmethod
    def _add_type(shape1_class, shape2_class):
        """
        This caches the per-contact type evaluation functions by
        using Contact._registers as a quick look-up for any two
        given shapes.

        To add more shapes, it's necessary to re-initialize the 
        registers, then call Contact._add_type for each collision
        type. The _registers are auto-sized to the amount of shapes
        (as per shapes.SHAPE_COUNT)
        """
        name1, name2 = shape1_class.__name__, shape2_class.__name__
        fcn_name = 'evaluate_%s_%s' % (name1.lower(), name2.lower())
        evaluate_fcn = getattr(Contact, fcn_name)
        
        type1, type2 = shape1_class._type, shape2_class._type
        Contact._registers[type1][type2].evaluate_fcn = evaluate_fcn
        Contact._registers[type1][type2].primary = True

        if type1 != type2:
            Contact._registers[type2][type1].evaluate_fcn = evaluate_fcn
            Contact._registers[type2][type1].primary = False

    @staticmethod
    def _initialize_registers():
        if Contact._initialized:
            return

        regs = []
        for i in range(len(shapes.Shapes)):
            regs.append([ContactRegister() for i in range(shapes.SHAPE_COUNT)])

        Contact._registers = regs
        Contact._initialized = True
        Contact._add_type(shapes.Circle, shapes.Circle)
        Contact._add_type(shapes.Polygon, shapes.Polygon)
        Contact._add_type(shapes.Polygon, shapes.Circle)
        Contact._add_type(shapes.Edge, shapes.Circle)
        Contact._add_type(shapes.Edge, shapes.Polygon)
        Contact._add_type(shapes.Loop, shapes.Circle)
        Contact._add_type(shapes.Loop, shapes.Polygon)

    @staticmethod
    def _create(fixture_a, index_a, fixture_b, index_b):
        type1 = fixture_a._shape._type
        type2 = fixture_b._shape._type

        reg = Contact._registers[type1][type2]
        #print('contact create %s/%s' % (fixture_a._shape.__class__.__name__, fixture_b._shape.__class__.__name__))
        if reg.evaluate_fcn:
            if reg.primary:
                return Contact(reg.evaluate_fcn, fixture_a, fixture_b, index_a, index_b)
            else:
                return Contact(reg.evaluate_fcn, fixture_b, fixture_a, index_b, index_a)
        else:
            return None

    @staticmethod
    def _destroy(contact, type_a=None, type_b=None):
        if contact._manifold.point_count > 0:
            contact.body_a.awake = True
            contact.body_b.awake = True

        type1 = contact._fixture_a._shape._type
        type2 = contact._fixture_b._shape._type

        reg = Contact._registers[type1][type2]
        if reg.destroy_fcn:
            reg.destroy_fcn(contact)
        else:
            return None

    def evaluate_circle_circle(self, manifold, xf_a, xf_b):
        collision.collide_circles(
            manifold, self._fixture_a.shape, xf_a, self._fixture_b.shape, xf_b)

    def evaluate_polygon_polygon(self, manifold, xf_a, xf_b):
        collision.collide_polygons(
            manifold, self._fixture_a.shape, xf_a, self._fixture_b.shape, xf_b)
        
    def evaluate_polygon_circle(self, manifold, xf_a, xf_b):
        collision.collide_polygon_circle(
            manifold, self._fixture_a.shape, xf_a, self._fixture_b.shape, xf_b)
    
    def evaluate_edge_circle(self, manifold, xf_a, xf_b):
        collision.collide_edge_circle(
            manifold, self._fixture_a.shape, xf_a, self._fixture_b.shape, xf_b)

    def evaluate_edge_polygon(self, manifold, xf_a, xf_b):
        collision.collide_edge_polygon(
            manifold, self._fixture_a.shape, xf_a, self._fixture_b.shape, xf_b)

    def evaluate_loop_circle(self, manifold, xf_a, xf_b):
        loop = self._fixture_a.shape
        edge = loop.child_edge(self._index_a)
        collision.collide_edge_circle(
            manifold, edge, xf_a, self._fixture_b.shape, xf_b)

    def evaluate_loop_polygon(self, manifold, xf_a, xf_b):
        loop = self._fixture_a.shape
        edge = loop.child_edge(self._index_a)
        collision.collide_edge_polygon(
            manifold, edge, xf_a, self._fixture_b.shape, xf_b)

class ContactSolver(object):
    def __init__(self, step, contacts, positions, velocities):
        self.step = step
        self.positions = positions
        self.velocities = velocities
        self.velocity_constraints = []
        self.position_constraints = []

        # Initialize position independent portions of the constraints.
        for contact in contacts:
            manifold = contact._manifold
            assert(manifold.point_count > 0)

            fixture_a = contact._fixture_a
            fixture_b = contact._fixture_b

            shape_a = fixture_a._shape
            shape_b = fixture_b._shape
            
            radius_a = shape_a.radius
            radius_b = shape_b.radius
            
            body_a = fixture_a._body
            body_b = fixture_b._body
            
            vc = ContactVelocityConstraint(contact, body_a, body_b)
            self.velocity_constraints.append(vc)

            pc = ContactPositionConstraint(body_a, body_b, radius_a, radius_b,
                                           manifold)
            self.position_constraints.append(pc)
           
            vcps = [VelocityConstraintPoint(step, mp.normal_impulse,
                                             mp.tangent_impulse)
                                    for mp in manifold.used_points]
           
            vc.points = vcps
            pc.local_points = [Vec2(*mp.local_point) for mp in manifold.used_points]

    def initialize_velocity_constraints(self):
        positions = self.positions
        velocities = self.velocities
        for vc, pc in zip(self.velocity_constraints, self.position_constraints):
            radius_a, radius_b = pc.radii
            manifold = vc.contact.manifold
            index_a, index_b = vc.indices

            ma, mb = vc.inv_mass
            ia, ib = vc.inv_i
            local_center_a, local_center_b = pc.local_centers
            
            ca, aa = positions[index_a] # center, angle
            cb, ab = positions[index_b]
            va, wa = velocities[index_a] # linear vel, angular vel
            vb, wb = velocities[index_b]

            assert(manifold.point_count > 0)
            
            xf_a = Transform(angle=aa)
            xf_b = Transform(angle=ab)
            xf_a.position = ca - xf_a._rotation * local_center_a
            xf_b.position = cb - xf_b._rotation * local_center_b

            world_manifold = WorldManifold(manifold, xf_a, radius_a, xf_b, radius_b)
            vc.normal = copy(world_manifold.normal)

            for vcp, wmp in zip(vc.points, world_manifold.points):
                ra, rb = vcp.r = (wmp - ca, wmp - cb)
               
                rn_a = ra.cross(vc.normal) ** 2
                rn_b = rb.cross(vc.normal) ** 2
                
                k_normal = ma + mb + ia * rn_a + ib * rn_b

                if k_normal > 0.0:
                    vcp.normal_mass = 1.0 / k_normal
                else:
                    vcp.normal_mass = 0.0

                tangent = vc.normal.cross(1.0)

                rt_a = ra.cross(tangent) ** 2
                rt_b = rb.cross(tangent) ** 2

                k_tangent = ma + mb + ia * rt_a + ib * rt_b

                if k_tangent > 0.0:
                    vcp.tangent_mass = 1.0 / k_tangent
                else:
                    vcp.tangent_mass = 0.0

                # Setup a velocity bias for restitution.
                vcp.velocity_bias = 0.0
                v_rel = vc.normal.dot(vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra))
                if v_rel < -VELOCITY_THRESHOLD:
                    vcp.velocity_bias = -vc.restitution * v_rel
            
            # If we have two points, then prepare the block solver.
            if len(vc.points) == 2:
                vcp1, vcp2 = vc.points

                ra1, rb1 = vcp1.r
                ra2, rb2 = vcp2.r
                rn1a = ra1.cross(vc.normal)
                rn1b = rb1.cross(vc.normal)
                rn2a = ra2.cross(vc.normal)
                rn2b = rb2.cross(vc.normal)

                k11 = ma + mb + ia * rn1a * rn1a + ib * rn1b * rn1b
                k22 = ma + mb + ia * rn2a * rn2a + ib * rn2b * rn2b
                k12 = ma + mb + ia * rn1a * rn2a + ib * rn1b * rn2b

                # Ensure a reasonable condition number.
                MAX_CONDITION_NUMBER = 1000.0 # TODO: settings
                if (k11 ** 2)  < (MAX_CONDITION_NUMBER * (k11 * k22 - k12 ** 2)):
                    # K is safe to invert.
                    vc.K = Mat22((k11, k12), (k12, k22))
                    vc.normal_mass = vc.K.inverse
                else:
                    # The constraints are redundant, just use one.
                    # TODO_ERIN use deepest? (upstream todo)
                    vc.points = [vc.points[0]]

    def warm_start(self):
        """Warm start"""
        positions = self.positions
        velocities = self.velocities
        for vc in self.velocity_constraints:
            inv_mass_a, inv_mass_b = vc.inv_mass
            inv_I_a, inv_I_b = vc.inv_i
            normal = vc.normal
            tangent = normal.cross(1.0)

            index_a, index_b = vc.indices

            ca, aa = positions[index_a] # center, angle
            cb, ab = positions[index_b]
            va, wa = velocities[index_a] # linear vel, angular vel
            vb, wb = velocities[index_b]
         
            for vcp in vc.points:
                p = vcp.normal_impulse * normal + vcp.tangent_impulse * tangent
                ra, rb = vcp.r
                wa -= inv_I_a * ra.cross(p)
                va -= inv_mass_a * p

                wb += inv_I_b * rb.cross(p)
                vb += inv_mass_b * p
            velocities[index_a] = (va, wa)
            velocities[index_b] = (vb, wb)

    def solve_velocity_constraints(self):
        positions = self.positions
        velocities = self.velocities
        for vc in self.velocity_constraints:
            index_a, index_b = vc.indices
            inv_mass_a, inv_mass_b = vc.inv_mass
            inv_I_a, inv_I_b = vc.inv_i
            normal = vc.normal
            tangent = normal.cross(1.0)

            ca, aa = positions[index_a] # center, angle
            cb, ab = positions[index_b]
            va, wa = velocities[index_a] # linear vel, angular vel
            vb, wb = velocities[index_b]
            
            normal = vc.normal
            tangent = normal.cross(1.0)
            friction = vc.friction

            assert(len(vc.points) in (1, 2))

            # Solve tangent constraints
            for vcp in vc.points:
                # Relative velocity at contact
                ra, rb = vcp.r
                dv = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)

                # Compute tangent force
                vt = dv.dot(tangent)
                lambda_ = vcp.tangent_mass * (-vt)

                # Clamp the accumulated force
                max_friction = friction * vcp.normal_impulse
                new_impulse = clamp(vcp.tangent_impulse + lambda_, -max_friction, max_friction)
                lambda_ = new_impulse - vcp.tangent_impulse

                # Apply contact impulse
                P = lambda_ * tangent

                va -= inv_mass_a * P
                wa -= inv_I_a * ra.cross(P)

                vb += inv_mass_b * P
                wb += inv_I_b * rb.cross(P)

                vcp.tangent_impulse = new_impulse

            # Solve normal constraints
            if len(vc.points) == 1:
                vcp = vc.points[0]
                # Relative velocity at contact
                ra, rb = vcp.r
                dv = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)

                # Compute normal impulse
                vn = dv.dot(normal)
                lambda_ = -vcp.normal_mass * (vn - vcp.velocity_bias)

                # clamp the accumulated impulse
                new_impulse = max(vcp.normal_impulse + lambda_, 0.0)
                lambda_ = new_impulse - vcp.normal_impulse

                # Apply contact impulse
                P = lambda_ * normal
                va -= inv_mass_a * P
                wa -= inv_I_a * ra.cross(P)

                vb += inv_mass_b * P
                wb += inv_I_b * rb.cross(P)
                vcp.normal_impulse = new_impulse
            else:
# Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D lite).
# Build the mini LCP for this contact patch
#
# vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
#
# A = J * W * J_t and J = ( -n, -r1 x n, n, r2 x n )
# b = vn_0 - velocity_bias
#
# The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
# implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2_d contact problem the cases
# vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
# solution that satisfies the problem is chosen.
# 
# In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
# that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
#
# Substitute:
# 
# x = x' - a
# 
# Plug into above equation:
#
# vn = A * x + b
#    = A * (x' - a) + b
#    = A * x' + b - A * a
#    = A * x' + b'
# b' = b - A * a

                cp1, cp2 = vc.points[:2]

                a = Vec2(cp1.normal_impulse, cp2.normal_impulse)
                assert(a.x >= 0.0 and a.y >= 0.0)

                # Relative velocity at contact
                ra1, rb1 = cp1.r
                ra2, rb2 = cp2.r
                dv1 = vb + scalar_cross(wb, rb1) - va - scalar_cross(wa, ra1)
                dv2 = vb + scalar_cross(wb, rb2) - va - scalar_cross(wa, ra2)

                # Compute normal velocity
                vn1 = dv1.dot(normal)
                vn2 = dv2.dot(normal)

                b = Vec2(vn1 - cp1.velocity_bias, vn2 - cp2.velocity_bias)
                b -= vc.K * a

                K_ERROR_TOL = 1e-3

                while True:
                    """
                     Case 1: vn = 0
                    
                     0 = A * x' + b'
                    
                     Solve for x':
                    
                     x' = - inv(A) * b'
                    """

                    x = -(vc.normal_mass * b)

                    if x.x >= 0.0 and x.y >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (ra1.cross(p1) + ra2.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (rb1.cross(p1) + rb2.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv1 = vb + scalar_cross(wb, rb1) - va - scalar_cross(wa, ra1)
                            dv2 = vb + scalar_cross(wb, rb2) - va - scalar_cross(wa, ra2)

                            # Compute normal velocity
                            vn1 = dv1.dot(normal)
                            vn2 = dv2.dot(normal)

                            assert(abs(vn1 - cp1.velocity_bias) < K_ERROR_TOL)
                            assert(abs(vn2 - cp2.velocity_bias) < K_ERROR_TOL)
                        break

                    """
                     Case 2: vn1 = 0 and x2 = 0
                    
                       0 = a11 * x1' + a12 * 0 + b1' 
                     vn2 = a21 * x1' + a22 * 0 + b2'
                    """

                    x = Vec2(-cp1.normal_mass * b.x, 0.0)
                    vn1 = 0.0
                    vn2 = vc.K.col1.y * x.x + b.y

                    if x.x >= 0.0 and vn2 >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (ra1.cross(p1) + ra2.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (rb1.cross(p1) + rb2.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv1 = vb + scalar_cross(wb, rb1) - va - scalar_cross(wa, ra1)

                            # Compute normal velocity
                            vn1 = dv1.dot(normal)

                            assert(abs(vn1 - cp1.velocity_bias) < K_ERROR_TOL)
                        break


                    """
                     Case 3: vn2 = 0 and x1 = 0
                    
                     vn1 = a11 * 0 + a12 * x2' + b1' 
                       0 = a21 * 0 + a22 * x2' + b2'
                    """
                    x = Vec2(0.0, -cp2.normal_mass * b.y)
                    vn1 = vc.K.col2.x * x.y + b.x
                    vn2 = 0.0

                    if x.y >= 0.0 and vn1 >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (ra1.cross(p1) + ra2.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (rb1.cross(p1) + rb2.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv2 = vb + scalar_cross(wb, rb2) - va - scalar_cross(wa, ra2)

                            # Compute normal velocity
                            vn2 = dv2.dot(normal)

                            assert(abs(vn2 - cp2.velocity_bias) < K_ERROR_TOL)
                        break

                    """
                     Case 4: x1 = 0 and x2 = 0
                     
                     vn1 = b1
                     vn2 = b2
                    """
                    x.x = 0.0
                    x.y = 0.0
                    vn1 = b.x
                    vn2 = b.y

                    if vn1 >= 0.0 and vn2 >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (ra1.cross(p1) + ra2.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (rb1.cross(p1) + rb2.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        break

                    # No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break

            self.velocities[index_a] = (va, wa)
            self.velocities[index_b] = (vb, wb)

    def store_impulses(self):
        for vc in self.velocity_constraints:
            m = vc.manifold
            for mp, vcp in zip(m.used_points, vc.points):
                mp.normal_impulse = vcp.normal_impulse
                mp.tangent_impulse = vcp.tangent_impulse


    def solve_position_constraints(self):
        """Sequential solver"""
        min_separation = 0.0
        positions = self.positions
        for pc in self.position_constraints:
            inv_mass_a, inv_mass_b = pc.inv_mass
            inv_I_a, inv_I_b = pc.inv_i
            local_center_a, local_center_b = pc.local_centers
            index_a, index_b = pc.indices

            ca, aa = positions[index_a] # center, angle
            cb, ab = positions[index_b]

            # Solve normal constraints
            for j in range(len(pc.local_points)):
                xf_a = Transform(angle=aa)
                xf_b = Transform(angle=ab)
                xf_a.position = ca - xf_a._rotation * local_center_a
                xf_b.position = cb - xf_b._rotation * local_center_b

                normal, point, separation = pc.solver_manifold(xf_a, xf_b, j)

                ra = point - ca
                rb = point - cb

                # Track max constraint error.
                min_separation = min(min_separation, separation)

                # Prevent large corrections and allow slop.
                C = clamp(BAUMGARTE * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0.0)

                # Compute the effective mass.
                rn_a = ra.cross(normal) ** 2
                rn_b = rb.cross(normal) ** 2
                K = inv_mass_a + inv_mass_b + (inv_I_a * rn_a) + (inv_I_b * rn_b)

                # Compute normal impulse
                if K > 0.0:
                    impulse = -C / K
                else:
                    impulse = 0.0

                P = impulse * normal

                ca -= inv_mass_a * P
                aa -= inv_I_a * ra.cross(P)

                cb += inv_mass_b * P
                ab += inv_I_b * rb.cross(P)
            
            positions[index_a] = (ca, aa)
            positions[index_b] = (cb, ab)

        # We can't expect min_speparation >= -LINEAR_SLOP because we don't
        # push the separation above -LINEAR_SLOP.
        return min_separation >= -3.0 * LINEAR_SLOP

    def solve_toi_position_constraints(self, toi_index_a, toi_index_b):
        """Sequential position solver for position constraints."""
        min_separation = 0.0
        positions = self.positions
        for pc in self.position_constraints:
            local_center_a, local_center_b = pc.local_centers
            index_a, index_b = pc.indices

            ca, aa = positions[index_a] # center, angle
            cb, ab = positions[index_b]

            if index_a in (toi_index_a, toi_index_b):
                inv_mass_a = pc.inv_mass[0]
                inv_I_a = pc.inv_i[0]
            else:
                inv_mass_a = 0.0
                inv_I_a = 0.0

            if index_b in (toi_index_a, toi_index_b):
                inv_mass_b = pc.inv_mass[1]
                inv_I_b = pc.inv_i[1]
            else:
                inv_mass_b = 0.0
                inv_I_b = 0.0

            # Solve normal constraints
            for j in range(len(pc.local_points)):
                xf_a = Transform(angle=aa)
                xf_b = Transform(angle=ab)
                xf_a.position = ca - xf_a._rotation * local_center_a
                xf_b.position = cb - xf_b._rotation * local_center_b

                normal, point, separation = pc.solver_manifold(xf_a, xf_b, j)

                ra = point - ca
                rb = point - cb

                # Track max constraint error.
                min_separation = min(min_separation, separation)

                # Prevent large corrections and allow slop.
                C = clamp(TOI_BAUMGARTE * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0.0)

                # Compute the effective mass.
                rn_a = ra.cross(normal) ** 2
                rn_b = rb.cross(normal) ** 2
                K = inv_mass_a + inv_mass_b + (inv_I_a * rn_a) + (inv_I_b * rn_b)

                # Compute normal impulse
                if K > 0.0:
                    impulse = -C / K
                else:
                    impulse = 0.0

                P = impulse * normal

                ca -= inv_mass_a * P
                aa -= inv_I_a * ra.cross(P)

                cb += inv_mass_b * P
                ab += inv_I_b * rb.cross(P)
            
            positions[index_a] = (ca, aa)
            positions[index_b] = (cb, ab)

        # We can't expect min_speparation >= -LINEAR_SLOP because we don't
        # push the separation above -LINEAR_SLOP.
        return min_separation >= -1.5 * LINEAR_SLOP

Contact._initialize_registers()
