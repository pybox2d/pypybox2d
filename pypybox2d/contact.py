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
from .common import (Vec2, Mat22, scalar_cross, clamp, distance_squared, property)
from .contact_util import (mix_friction, mix_restitution, 
                           Manifold, WorldManifold, 
                           ContactRegister, ContactConstraint)

EPSILON = settings.EPSILON
EPSILON_SQR = settings.EPSILON_SQR
VELOCITY_THRESHOLD = settings.VELOCITY_THRESHOLD
DEBUG_SOLVER = settings.DEBUG_SOLVER
LINEAR_SLOP = settings.LINEAR_SLOP
MAX_LINEAR_CORRECTION = settings.MAX_LINEAR_CORRECTION

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
        raise mix_friction(self._fixture_a._friction, self._fixture_b._friction)

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
        raise mix_restitution(self._fixture_a._restitution, self._fixture_b._restitution)

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
            for mp2 in self._manifold.used_points:
                mp2.normal_impulse = 0.0
                mp2.tangent_impulse = 0.0
                id2 = mp2.id
                for mp1 in old_manifold.used_points:
                    if mp1.id == id2:
                        mp2.normal_impulse = mp1.normal_impulse
                        mp2.tangent_impulse = mp1.tangent_impulse
                        break

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
    def __init__(self, contacts, impulse_ratio, warm_starting):
        self.count = len(contacts)
        self.constraints = []
        
        # Initialize position independent portions of the constraints.
        for contact in contacts:
            fixture_a = contact._fixture_a
            fixture_b = contact._fixture_b

            shape_a = fixture_a._shape
            shape_b = fixture_b._shape

            radius_a = shape_a.radius
            radius_b = shape_b.radius
            
            body_a = fixture_a._body
            body_b = fixture_b._body

            manifold = contact._manifold

            assert(manifold.point_count > 0)

            constraint = ContactConstraint(manifold.point_count)
            self.constraints.append(constraint)

            constraint.friction = contact._friction
            constraint.restitution = contact._restitution
            constraint.body_a = body_a
            constraint.body_b = body_b

            constraint.manifold = manifold
            constraint.normal = Vec2()

            constraint.local_normal = copy(manifold.local_normal)
            constraint.local_point = copy(manifold.local_point)
            constraint.radius_a = radius_a
            constraint.radius_b = radius_b
            constraint.type = manifold.type

            assert(len(manifold.used_points) == len(constraint.points)) # TODO REMOVE
            for mp, ccp in zip(manifold.used_points, constraint.points):
                if warm_starting:
                    ccp.normal_impulse = impulse_ratio * mp.normal_impulse
                    ccp.tangent_impulse = impulse_ratio * mp.tangent_impulse
                else:
                    ccp.normal_impulse = 0.0
                    ccp.tangent_impulse = 0.0
                
                ccp.local_point = copy(mp.local_point)
                ccp.ra = Vec2()
                ccp.rb = Vec2()
                ccp.normal_mass = 0.0
                ccp.tangent_mass = 0.0
                ccp.velocity_bias = 0.0
            
            constraint.K = Mat22()
            constraint.normal_mass = Vec2()

    def initialize_velocity_constraints(self):
        for cc in self.constraints:
            radius_a = cc.radius_a
            radius_b = cc.radius_b
            body_a = cc.body_a
            body_b = cc.body_b
            manifold = cc.manifold

            va = body_a._linear_velocity
            vb = body_b._linear_velocity
            wa = body_a._angular_velocity
            wb = body_b._angular_velocity

            assert(manifold.point_count > 0)

            world_manifold = WorldManifold(manifold, body_a._xf, radius_a, body_b._xf, radius_b)
            cc.normal = copy(world_manifold.normal)
            assert(len(cc.points) == len(world_manifold.points)) # TODO REMOVE
            for ccp, wmp in zip(cc.points, world_manifold.points):
                ccp.ra = wmp - body_a._sweep.c
                ccp.rb = wmp - body_b._sweep.c
                
                rn_a = ccp.ra.cross(cc.normal) ** 2
                rn_b = ccp.rb.cross(cc.normal) ** 2
                
                k_normal = body_a._inv_mass + body_b._inv_mass + body_a._invI * rn_a + body_b._invI * rn_b

                assert(k_normal > EPSILON)

                ccp.normal_mass = 1.0 / k_normal
                tangent = cc.normal.cross(1.0)

                rt_a = ccp.ra.cross(tangent) ** 2
                rt_b = ccp.rb.cross(tangent) ** 2

                k_tangent = body_a._inv_mass + body_b._inv_mass + body_a._invI * rt_a + body_b._invI * rt_b

                assert(k_tangent > EPSILON)
                ccp.tangent_mass = 1.0 / k_tangent

                # Setup a velocity bias for restitution.
                ccp.velocity_bias = 0.0
                v_rel = cc.normal.dot(vb + scalar_cross(wb, ccp.rb) - va - scalar_cross(wa, ccp.ra))
                if v_rel < -VELOCITY_THRESHOLD:
                    ccp.velocity_bias = -cc.restitution * v_rel
            

            # If we have two points, then prepare the block solver.
            if cc.point_count == 2:
                ccp1 = cc.points[0]
                ccp2 = cc.points[1]

                inv_mass_a = body_a._inv_mass
                inv_mass_b = body_b._inv_mass
                inv_I_a = body_a._invI
                inv_I_b = body_b._invI
                
                rn1a = ccp1.ra.cross(cc.normal)
                rn1b = ccp1.rb.cross(cc.normal)
                rn2a = ccp2.ra.cross(cc.normal)
                rn2b = ccp2.rb.cross(cc.normal)

                k11 = inv_mass_a + inv_mass_b + inv_I_a * rn1a * rn1a + inv_I_b * rn1b * rn1b
                k22 = inv_mass_a + inv_mass_b + inv_I_a * rn2a * rn2a + inv_I_b * rn2b * rn2b
                k12 = inv_mass_a + inv_mass_b + inv_I_a * rn1a * rn2a + inv_I_b * rn1b * rn2b

                # Ensure a reasonable condition number.
                MAX_CONDITION_NUMBER = 1000.0
                if (k11 ** 2)  < (MAX_CONDITION_NUMBER * (k11 * k22 - k12 ** 2)):
                    # K is safe to invert.
                    cc.K.col1 = Vec2(k11, k12)
                    cc.K.col2 = Vec2(k12, k22)
                    cc.normal_mass = cc.K.inverse
                else:
                    # The constraints are redundant, just use one.
                    # TODO_ERIN use deepest? (upstream todo)
                    cc.point_count = 1

    def warm_start(self):
        """Warm start"""
        for c in self.constraints:
            body_a = c.body_a
            body_b = c.body_b
            inv_mass_a = body_a._inv_mass
            inv_mass_b = body_b._inv_mass
            inv_I_a = body_a._invI
            inv_I_b = body_b._invI
            normal = c.normal
            tangent = normal.cross(1.0)

            for ccp in c.points:
                p = ccp.normal_impulse * normal + ccp.tangent_impulse * tangent
                body_a._angular_velocity -= inv_I_a * ccp.ra.cross(p)
                body_a._linear_velocity -= inv_mass_a * p

                body_b._angular_velocity += inv_I_b * ccp.rb.cross(p)
                body_b._linear_velocity += inv_mass_b * p

    def solve_velocity_constraints(self):
        for c in self.constraints:
            body_a = c.body_a
            body_b = c.body_b
            va = body_a._linear_velocity # note that we modify these in place
            vb = body_b._linear_velocity # note that we modify these in place
            wa = body_a._angular_velocity
            wb = body_b._angular_velocity
            inv_mass_a = body_a._inv_mass
            inv_mass_b = body_b._inv_mass
            inv_I_a = body_a._invI
            inv_I_b = body_b._invI
            normal = c.normal
            tangent = normal.cross(1.0)
            friction = c.friction
            point_count = c.point_count

            assert(point_count in (1, 2))

            # Solve tangent constraints
            for ccp in c.points:
                # Relative velocity at contact
                dv = vb + scalar_cross(wb, ccp.rb) - va - scalar_cross(wa, ccp.ra)

                # Compute tangent force
                vt = dv.dot(tangent)
                lambda_ = ccp.tangent_mass * (-vt)

                # Clamp the accumulated force
                max_friction = friction * ccp.normal_impulse
                new_impulse = clamp(ccp.tangent_impulse + lambda_, -max_friction, max_friction)
                lambda_ = new_impulse - ccp.tangent_impulse

                # Apply contact impulse
                P = lambda_ * tangent

                va -= inv_mass_a * P
                wa -= inv_I_a * ccp.ra.cross(P)

                vb += inv_mass_b * P
                wb += inv_I_b * ccp.rb.cross(P)

                ccp.tangent_impulse = new_impulse

            # Solve normal constraints
            if c.point_count == 1:
                ccp = c.points[0]

                # Relative velocity at contact
                dv = vb + scalar_cross(wb, ccp.rb) - va - scalar_cross(wa, ccp.ra)

                # Compute normal impulse
                vn = dv.dot(normal)
                lambda_ = -ccp.normal_mass * (vn - ccp.velocity_bias)

                # clamp the accumulated impulse
                new_impulse = max(ccp.normal_impulse + lambda_, 0.0)
                lambda_ = new_impulse - ccp.normal_impulse

                # Apply contact impulse
                P = lambda_ * normal
                va -= inv_mass_a * P
                wa -= inv_I_a * ccp.ra.cross(P)

                vb += inv_mass_b * P
                wb += inv_I_b * ccp.rb.cross(P)
                ccp.normal_impulse = new_impulse
            else:
                """
                 Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2_d__lite).
                 Build the mini L_c_p for this contact patch
                
                 vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                
                 A = J * W * J_t and J = ( -n, -r1 x n, n, r2 x n )
                 b = vn_0 - velocity_bias
                
                 The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                 implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2_d contact problem the cases
                 vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                 solution that satisfies the problem is chosen.
                 
                 In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                 that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                
                 Substitute:
                 
                 x = x' - a
                 
                 Plug into above equation:
                
                 vn = A * x + b
                    = A * (x' - a) + b
                    = A * x' + b - A * a
                    = A * x' + b'
                 b' = b - A * a
                """

                cp1 = c.points[0]
                cp2 = c.points[1]

                a = Vec2(cp1.normal_impulse, cp2.normal_impulse)
                assert(a.x >= 0.0 and a.y >= 0.0)

                # Relative velocity at contact
                dv1 = vb + scalar_cross(wb, cp1.rb) - va - scalar_cross(wa, cp1.ra)
                dv2 = vb + scalar_cross(wb, cp2.rb) - va - scalar_cross(wa, cp2.ra)

                # Compute normal velocity
                vn1 = dv1.dot(normal)
                vn2 = dv2.dot(normal)

                b = Vec2(vn1 - cp1.velocity_bias, vn2 - cp2.velocity_bias)
                b -= c.K * a

                K_ERROR_TOL = 1e-3

                while True:
                    """
                     Case 1: vn = 0
                    
                     0 = A * x' + b'
                    
                     Solve for x':
                    
                     x' = - inv(A) * b'
                    """

                    x = -(c.normal_mass * b)

                    if x.x >= 0.0 and x.y >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (cp1.ra.cross(p1) + cp2.ra.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (cp1.rb.cross(p1) + cp2.rb.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv1 = vb + scalar_cross(wb, cp1.rb) - va - scalar_cross(wa, cp1.ra)
                            dv2 = vb + scalar_cross(wb, cp2.rb) - va - scalar_cross(wa, cp2.ra)

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
                    vn2 = c.K.col1.y * x.x + b.y

                    if x.x >= 0.0 and vn2 >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (cp1.ra.cross(p1) + cp2.ra.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (cp1.rb.cross(p1) + cp2.rb.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv1 = vb + scalar_cross(wb, cp1.rb) - va - scalar_cross(wa, cp1.ra)

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
                    vn1 = c.K.col2.x * x.y + b.x
                    vn2 = 0.0

                    if x.y >= 0.0 and vn1 >= 0.0:
                        # Resubstitute for the incremental impulse
                        d = x - a

                        # Apply incremental impulse
                        p1 = d.x * normal
                        p2 = d.y * normal
                        va -= inv_mass_a * (p1 + p2)
                        wa -= inv_I_a * (cp1.ra.cross(p1) + cp2.ra.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (cp1.rb.cross(p1) + cp2.rb.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        if DEBUG_SOLVER:
                            # Postconditions
                            dv2 = vb + scalar_cross(wb, cp2.rb) - va - scalar_cross(wa, cp2.ra)

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
                        wa -= inv_I_a * (cp1.ra.cross(p1) + cp2.ra.cross(p2))

                        vb += inv_mass_b * (p1 + p2)
                        wb += inv_I_b * (cp1.rb.cross(p1) + cp2.rb.cross(p2))

                        # Accumulate
                        cp1.normal_impulse = x.x
                        cp2.normal_impulse = x.y

                        break

                    # No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break

            body_a._angular_velocity = wa
            body_b._angular_velocity = wb

    def store_impulses(self):
        for c in self.constraints:
            m = c.manifold
            assert(len(m.used_points) == len(c.points)) # TODO REMOVE
            for mp, ccp in zip(m.used_points, c.points):
                mp.normal_impulse = ccp.normal_impulse
                mp.tangent_impulse = ccp.tangent_impulse

    def position_solver_manifold(self, cc, ccp):
        """
        Constraint cc, ContactConstraintPoint ccp
        returns: normal, point, separation
        """
        if cc.type == Manifold.CIRCLES:
            point_a = cc.body_a.get_world_point(cc.local_point)
            point_b = cc.body_b.get_world_point(ccp.local_point)
            if distance_squared(point_a, point_b) > EPSILON_SQR:
                normal = point_b - point_a
                normal.normalize()
            else:
                normal = Vec2(1.0, 0.0)

            point = 0.5 * (point_a + point_b)
            separation = (point_b - point_a).dot(normal) - cc.radius_a - cc.radius_b

        elif cc.type == Manifold.FACE_A:
            normal = cc.body_a.get_world_vector(cc.local_normal)
            plane_point = cc.body_a.get_world_point(cc.local_point)

            clip_point = cc.body_b.get_world_point(ccp.local_point)
            separation = (clip_point - plane_point).dot(normal) - cc.radius_a - cc.radius_b
            point = clip_point

        elif cc.type == Manifold.FACE_B:
            normal = cc.body_b.get_world_vector(cc.local_normal)
            plane_point = cc.body_b.get_world_point(cc.local_point)

            clip_point = cc.body_a.get_world_point(ccp.local_point)
            separation = (clip_point - plane_point).dot(normal) - cc.radius_a - cc.radius_b
            point = clip_point

            # Ensure normal points from A to B
            normal = -normal

        return normal, point, separation

    def solve_position_constraints(self, baumgarte):
        """Sequential solver"""
        min_separation = 0.0
        for c in self.constraints:
            body_a = c.body_a
            body_b = c.body_b
            inv_mass_a = body_a._mass * body_a._inv_mass
            inv_mass_b = body_b._mass * body_b._inv_mass
            inv_I_a = body_a._mass * body_a._invI
            inv_I_b = body_b._mass * body_b._invI

            # Solve normal constraints
            for ccp in c.points:
                normal, point, separation = self.position_solver_manifold(c, ccp)

                r_a = point - body_a._sweep.c
                r_b = point - body_b._sweep.c

                # Track max constraint error.
                min_separation = min(min_separation, separation)

                # Prevent large corrections and allow slop.
                C = clamp(baumgarte * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0.0)

                # Compute the effective mass.
                rn_a = r_a.cross(normal)
                rn_b = r_b.cross(normal)
                K = inv_mass_a + inv_mass_b + (inv_I_a * rn_a * rn_a) + (inv_I_b * rn_b * rn_b)

                # Compute normal impulse
                if K > 0.0:
                    impulse = -C / K
                else:
                    impulse = 0.0

                P = impulse * normal

                body_a._sweep.c -= inv_mass_a * P
                body_a._sweep.a -= inv_I_a * r_a.cross(P)
                body_a._synchronize_transform()

                body_b._sweep.c += inv_mass_b * P
                body_b._sweep.a += inv_I_b * r_b.cross(P)
                body_b._synchronize_transform()

        # We can't expect min_speparation >= -LINEAR_SLOP because we don't
        # push the separation above -LINEAR_SLOP.
        return min_separation >= -1.5 * LINEAR_SLOP

    def solve_toi_position_constraints(self, baumgarte, toi_body_a, toi_body_b):
        """Sequential position solver for position constraints."""
        return self.solve_position_constraints(baumgarte)
        # Porting note: this is mostly identical to solve_position_constraints, but
        #       with the addition of unused toi_bodies and unused mass values --

        #min_separation = 0.0
        #for c in self.constraints:
        #    body_a = c.body_a
        #    body_b = c.body_b

        #    if body_a == toi_body_a or body_a == toi_body_b:
        #        mass_a = body_a._mass
        #    else:
        #        mass_a = 0.0

        #    if body_b == toi_body_a or body_b == toi_body_b:
        #        mass_b = body_b._mass
        #    else:
        #        mass_b = 0.0

        #    # the rest is identical to solve_position_constraints...

Contact._initialize_registers()
