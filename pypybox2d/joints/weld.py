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

__all__ = ('WeldJoint', )

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Vec3, Mat22, Mat33, scalar_cross, property)
from ..settings import (LINEAR_SLOP, ANGULAR_SLOP)
from .joint import Joint

class WeldJoint(Joint):
    """
    A weld joint essentially glues two bodies together. A weld joint may
    distort somewhat because the island constraint solver is approximate.
    
    In creating one, you need to specify either one world anchor point or
    two local anchor points where they are attached and the relative body
    angle. The position of the anchor points is important for computing 
    the reaction torque.
    """
    # Point-to-point constraint
    # C = p2 - p1
    # Cdot = vb - va
    #      = vb + cross(wb, rb) - va - cross(wa, ra)
    # J = [-I -ra_skew I rb_skew ]
    # Identity used:
    # w k % (rx i + ry j) = w * (-ry i + rx j)

    # Angle constraint
    # C = angle2 - angle1 - referenceAngle
    # Cdot = wb - wa
    # J = [0 0 -1 0 0 1]
    # K = invIa + invIb
    def __init__(self, body_a, body_b, anchor=None, reference_angle=None,
                 local_anchor_a=None, local_anchor_b=None,
                 collide_connected=False):
        """

        """
        if body_a is None or body_b is None:
            raise ValueError('body_a and body_b must be set')

        Joint.__init__(self, body_a, body_b, collide_connected)

        if anchor is not None:
            self._local_anchor_a = body_a.get_local_point(anchor)
            self._local_anchor_b = body_b.get_local_point(anchor)
        else:
            self._local_anchor_a = Vec2(*local_anchor_a)
            self._local_anchor_b = Vec2(*local_anchor_b)
  
        if reference_angle is not None:
            self._reference_angle = float(reference_angle)
        else:
            self._reference_angle = body_b.angle - body_a.angle

        self._mass = Mat33()
        self._impulse = Vec3()

    def __copy__(self):
        return WeldJoint(self._body_a, self._body_b, None, self._reference_angle,
                         self._local_anchor_a, self._local_anchor_b,
                         self._collide_connected)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * Vec2(self._impulse.x, self._impulse.y)

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        return inv_dt * self._impulse.z

    @property
    def anchor_a(self):
        """Get the anchor point on body_a in world coordinates"""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """Get the anchor point on body_b in world coordinates"""
        return self._body_b.get_world_point(self._local_anchor_b)

    def _init_velocity_constraints(self, step, positions, velocities):
        body_a, body_b = self._body_a, self._body_b
        index_a = body_a._island_index
        index_b = body_b._island_index
        self._indices = (index_a, index_b)

        local_center_a = self._local_center_a = body_a._sweep.local_center
        local_center_b = self._local_center_b = body_b._sweep.local_center

        ma = self._inv_mass_a = body_a._inv_mass
        mb = self._inv_mass_b = body_b._inv_mass

        ia = self._inv_Ia = body_a._invI
        ib = self._inv_Ib = body_b._invI

        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        # Compute the effective mass matrix.
        ra = self._ra = qa * (self._local_anchor_a - local_center_a)
        rb = self._rb = qb * (self._local_anchor_b - local_center_b)

        # J = [-I -ra_skew I rb_skew]
        #     [ 0       -1 0       1]
        # r_skew = [-ry; rx]

        # Matlab
        # K = [ mA+ray^2*iA+mB+rby^2*iB,  -ray*iA*rax-rby*iB*rbx,          -ray*iA-rby*iB]
        #     [  -ray*iA*rax-rby*iB*rbx, mA+rax^2*iA+mB+rbx^2*iB,           rax*iA+rbx*iB]
        #     [          -ray*iA-rby*iB,           rax*iA+rbx*iB,                   iA+iB]

        c2x = -ra.y * ra.x * ia - rb.y * rb.x * ib
        c3x = -ra.y * ia - rb.y * ib
        self._mass.col1 = Vec3(ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib,
                               c2x,
                               c3x)
        self._mass.col2 = Vec3(c2x,
                               ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib,
                               c3x)
        self._mass.col3 = Vec3(c3x,
                               ra.x * ia + rb.x * ib,
                               ia + ib)

        if step.warm_starting:
            # Scale impulses to support a variable time step.
            self._impulse *= step.dt_ratio

            ix, iy, iz = self._impulse
            P = Vec2(ix, iy)

            va -= ma * P
            wa -= ia * (ra.cross(P) + iz)

            vb += mb * P
            wb += ib * (rb.cross(P) + iz)
        else:
            self._impulse = Vec3()
        
        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_velocity_constraints(self, step, positions, velocities):
        index_a, index_b = self._indices

        va, wa = velocities[index_a]
        vb, wb = velocities[index_b]

        ra, rb = self._ra, self._rb
        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Solve point-to-point constraint
        Cdot1 = vb + scalar_cross(wb, rb) - va - scalar_cross(wa, ra)
        Cdot2 = wb - wa
        Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2)

        impulse = self._mass.solve3x3(-Cdot)
        self._impulse += impulse

        P = Vec2(impulse.x, impulse.y)

        va -= ma * P
        wa -= ia * (ra.cross(P) + impulse.z)

        vb += mb * P
        wb += ib * (rb.cross(P) + impulse.z)

        velocities[index_a] = (va, wa)
        velocities[index_b] = (vb, wb)

    def _solve_position_constraints(self, step, positions, velocities):
        """This returns true if the position errors are within tolerance."""
        index_a, index_b = self._indices
        ca, aa = positions[index_a]
        cb, ab = positions[index_b]

        qa = Mat22(angle=aa)
        qb = Mat22(angle=ab)

        ma, mb = self._inv_mass_a, self._inv_mass_b
        ia, ib = self._inv_Ia, self._inv_Ib

        # Compute the effective mass matrix.
        ra = qa * (self._local_anchor_a - self._local_center_a)
        rb = qb * (self._local_anchor_b - self._local_center_b)

        C1 = cb + rb - ca - ra
        C2 = ab - aa - self._reference_angle

        position_error = C1.length
        angular_error = abs(C2)

        mass = self._mass


        c2x = -ra.y * ra.x * ia - rb.y * rb.x * ib
        c3x = -ra.y * ia - rb.y * ib
        c3y = ra.x * ia + rb.x * ib

        mass.col1 = Vec3(ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib,
                         c2x,
                         c3x)

        mass.col2 = Vec3(c2x,
                         ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib,
                         c3y)

        mass.col3 = Vec3(-ra.y * ia - rb.y * ib,
                          ra.x * ia + rb.x * ib,
                          ia + ib)

        C = Vec3(C1.x, C1.y, C2)
        impulse = -mass.solve3x3(C)

        ix, iy, iz = impulse
        P = Vec2(ix, iy)
        ca -= ma * P
        aa -= ia * (ra.cross(P) + iz)

        cb += mb * P
        ab += ib * (rb.cross(P) + iz)

        positions[index_a] = (ca, aa)
        positions[index_b] = (cb, ab)

        return position_error <= LINEAR_SLOP and angular_error <= ANGULAR_SLOP

