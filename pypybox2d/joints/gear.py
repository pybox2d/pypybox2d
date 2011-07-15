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

__all__ = ('Joint', 'FrictionJoint', 
           'PrismaticJoint', 'RopeJoint', 'WheelJoint', 
           'MouseJoint', 'PulleyJoint', 'GearJoint',
          )

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from ..common import (Vec2, Mat22, is_valid_float, property)
from .joint import Joint
from .revolute import RevoluteJoint
from .prismatic import PrismaticJoint

class GearJoint(Joint):
    """
    A gear joint is used to connect two joints together. Either joint
    can be a revolute or prismatic joint. You specify a gear ratio
    to bind the motions together:
    coordinate1 + ratio * coordinate2 = constant
    The ratio can be negative or positive. If one joint is a revolute joint
    and the other joint is a prismatic joint, then the ratio will have units
    of length or units of 1/length.
    @warning The revolute and prismatic joints must be attached to
    fixed bodies (which _must be_ body_a on those joints).

    Creation requires two existing
    revolute or prismatic joints (any combination will work).
    The provided joints must attach a dynamic body to a static body.
    """
    # Gear Joint:
    # C0 = (coordinate1 + ratio * coordinate2)_initial
    # C = C0 - (cordinate1 + ratio * coordinate2) = 0
    # Cdot = -(Cdot1 + ratio * Cdot2)
    # J = -[J1 ratio * J2]
    # K = J * invM * JT
    #   = J1 * invMa * J1T + ratio * ratio * J2 * invMb * J2T
    #
    # Revolute:
    # coordinate = rotation
    # Cdot = angularVelocity
    # J = [0 0 1]
    # K = J * invM * JT = invI
    #
    # Prismatic:
    # coordinate = dot(p - pg, ug)
    # Cdot = dot(v + cross(w, r), ug)
    # J = [ug cross(r, ug)]
    # K = J * invM * JT = invMass + invI * cross(r, ug)^2
    def __init__(self, joint_a, joint_b, ratio=0.1):
        Joint.__init__(self, None, None, False)

        if not isinstance(joint_a, (RevoluteJoint, PrismaticJoint)):
            raise ValueError('Joints must be either Revolute or Prismatic')
        if not isinstance(joint_b, (RevoluteJoint, PrismaticJoint)):
            raise ValueError('Joints must be either Revolute or Prismatic')

        self._joint_a = joint_a
        self._joint_b = joint_b

        # Get geometry of joint_a
        body_a = self._body_a = joint_a.body_b
        body_c = self._body_c = joint_a.body_a

        xf_a, xf_c = body_a._xf, body_c._xf
        aa, ac = body_a._sweep.a, body_c._sweep.a

        self._revolute_a = isinstance(joint_a, RevoluteJoint)
        if self._revolute_a:
            revolute = joint_a
            self._local_anchor_c = Vec2(*revolute._local_anchor_a)
            self._local_anchor_a = Vec2(*revolute._local_anchor_b)
            self._reference_angle_a = revolute._reference_angle
            self._local_axis_c = Vec2()

            coordinate_a = aa - ac - self._reference_angle_a
        else:
            prismatic = joint_a
            self._local_anchor_c = Vec2(*prismatic._local_anchor_a)
            self._local_anchor_a = Vec2(*prismatic._local_anchor_b)
            self._reference_angle_a = prismatic._reference_angle
            self._local_axis_c = Vec2(*prismatic._local_x_axis)
            
            pc = self._local_anchor_c
            pa = xf_c.rotation.mul_t(xf_a.rotation * self._local_anchor_a + (xf_a.position - xf_c.position))
            coordinate_a = (pa - pc).dot(self._local_axis_c)

        # Get geometry of joint_b
        body_d = self._body_d = joint_a.body_a
        body_b = self._body_b = joint_a.body_b

        xf_b, xf_d = body_b._xf, body_d._xf
        ab, ad = body_b._sweep.a, body_d._sweep.a
        self._revolute_b = isinstance(joint_b, RevoluteJoint)
        if self._revolute_b:
            revolute = joint_b
            self._local_anchor_d = Vec2(*revolute._local_anchor_a)
            self._local_anchor_b = Vec2(*revolute._local_anchor_b)
            self._reference_angle_b = revolute._reference_angle
            self._local_axis_d = Vec2()

            coordinate_b = ab - ad - self._reference_angle_b
        else:
            prismatic = joint_b
            self._local_anchor_d = Vec2(*prismatic._local_anchor_a)
            self._local_anchor_b = Vec2(*prismatic._local_anchor_b)
            self._reference_angle_b = prismatic._reference_angle
            self._local_axis_d = Vec2(*prismatic._local_x_axis)
            
            pd = self._local_anchor_d
            pb = xf_d.rotation.mul_t(xf_b.rotation * self._local_anchor_b + (xf_b.position - xf_d.position))
            coordinate_b = (pb - pd).dot(self._local_axis_d)

        self._ground_anchor_b = Vec2(*joint_b._local_anchor_a)
        self._local_anchor_b = Vec2(*joint_b._local_anchor_b)
        
        self._ratio = ratio
        self._constant = coordinate_a + self._ratio * coordinate_b
        self._impulse = 0.0
        self._bodies = (self._body_a, self._body_b, self._body_c, self._body_d)

    def __copy__(self):
        return GearJoint(self._joint_a, self._joint_b, self._ratio)

    def get_reaction_force(self, inv_dt):
        """Get the reaction force on body_b at the joint anchor in Newtons."""
        return inv_dt * (self._impulse * self._j.linear_b) # TODO_ERIN not tested

    def get_reaction_torque(self, inv_dt):
        """Get the reaction torque on body_b in N*m."""
        r = self._body_b._xf._rotation * (self._local_anchor_b - self._body_b._sweep.local_center)
        P = self._impulse * self._j.linear_b
        L = self._impulse * self._j.angular_b - r.cross(P)
        return inv_dt * L # TODO_ERIN not tested

    @property
    def joint_a(self):
        return self._joint_a

    @property
    def joint_b(self):
        return self._joint_b

    @property
    def revolute_joint_a(self):
        return self._revolute_a

    @property
    def revolute_joint_b(self):
        return self._revolute_b

    @property
    def prismatic_joint_a(self):
        return self._prismatic_a

    @property
    def prismatic_joint_b(self):
        return self._prismatic_b

    @property
    def ratio(self):
        return self._ratio

    @ratio.setter
    def ratio(self, ratio):
        if not is_valid_float(ratio):
            raise ValueError('Invalid ratio')
        self._ratio = ratio

    @property
    def anchor_a(self):
        """The world anchor point relative to body_a's origin."""
        return self._body_a.get_world_point(self._local_anchor_a)

    @property
    def anchor_b(self):
        """The world anchor point relative to body_b's origin."""
        return self._body_b.get_world_point(self._local_anchor_b)

    def _init_velocity_constraints(self, step, positions, velocities):
        bodies = self._bodies
        self._indices = [body._island_index for body in bodies]
        self._local_centers = [body._sweep.local_center for body in bodies]
        self._masses = [body._inv_mass for body in bodies]
        self._inertias = [body._invI for body in bodies]

        index_a, index_b, index_c, index_d = indices = self._indices
        Cs = self._centers = [positions[index][0] for index in indices]
        As = self._angles = [positions[index][1] for index in indices]

        Vs = self._lin_vels = [velocities[index][0] for index in indices]
        Ws = self._ang_vels = [velocities[index][1] for index in indices]

        qa, qb, qc, qd = [Mat22(angle=a) for a in As]
        ca, cb, cc, cd = Cs
        aa, ab, ac, ad = As
        ia, ib, ic, id_ = self._inertias
        ma, mb, mc, md = self._masses
        lca, lcb, lcc, lcd = self._local_centers

        self._mass = 0.0

        if self._revolute_a:
            self._j_vac = Vec2()
            self._j_wa = 1.0
            self._j_wc = 1.0
            self._mass += ia + ic
        else:
            u = qc * self._local_axis_c
            rc = qc * (self._local_anchor_c - lcc)
            ra = qa * (self._local_anchor_a - lca)
            self._j_vac = u
            self._j_wc = rc.cross(u)
            self._j_wa = ra.cross(u)
            self._mass += mc + ma + ic * self._j_wc * self._j_wc + ia * self._j_wa * self._j_wa

        if self._revolute_b:
            self._j_vbd = Vec2()
            self._j_wb = 1.0
            self._j_wd = 1.0
            self._mass += self._ratio**2 * (ib + id_)
        else:
            u = qd * self._local_axis_d
            rd = qd * (self._local_anchor_d - lcd)
            rb = qb * (self._local_anchor_b - lcb)
            self._j_vbd = u
            self._j_wd = rd.cross(u)
            self._j_wb = rb.cross(u)
            self._mass += self._ratio**2 * (md + mb) + id_ * self._j_wd ** 2 + ib * self._j_wb * self._j_wb

        # Compute effective mass.
        if self._mass > 0.0:
            self._mass = 1.0 / self._mass
        else:
            self._mass = 0.0

        # Linear velocity Jacobian values corresponding to lin vel a~d
        self._lv_j_values = [self._j_vac, self._j_vbd, -self._j_vac, -self._j_vbd]

        # Angular velocity Jacobian values corresponding to ang vel a~d
        self._av_j_values = [self._j_wa, self._j_wb, -self._j_wc, -self._j_wd]

        if step.warm_starting:
            # Warm starting.
            impulse = self._impulse
            #for k, (i, m, lvj, avj) in enumerate(zip(self._inertias, self._masses, self._lv_j_values, self._av_j_values)):
            #    Vs[k] += (m * impulse) * lvj
            #    Ws[k] += (i * impulse) * avj

            Vs[0] += (ma * impulse) * self._j_vac
            Ws[0] += ia * impulse * self._j_wa

            Vs[1] += (mb * impulse) * self._j_vbd
            Ws[1] += ib * impulse * self._j_wb

            Vs[2] -= (mc * impulse) * self._j_vac
            Ws[2] -= ic * impulse * self._j_wc

            Vs[3] -= (md * impulse) * self._j_vbd
            Ws[3] -= id_ * impulse * self._j_wd
        else:
            self._impulse = 0.0

        for index, v, w in zip(self._indices, Vs, Ws):
            velocities[index] = (v, w)

    def _solve_velocity_constraints(self, step, positions, velocities):
        indices = self._indices
        Vs = self._lin_vels
        Ws = self._ang_vels = [velocities[index][1] for index in indices]

        va, vb, vc, vd = Vs
        wa, wb, wc, wd = Ws

        Cdot = self._j_vac.dot(va - vc) + self._j_vbd.dot(vb - vd)
        Cdot += (self._j_wa * wa - self._j_wc * wc) + (self._j_wb * wb - self._j_wd * wd)

        impulse = -self._mass * Cdot
        self._impulse += impulse

        for k, (i, m, lvj, avj) in enumerate(zip(self._inertias, self._masses, self._lv_j_values, self._av_j_values)):
            Vs[k] += (m * impulse) * lvj
            Ws[k] += (i * impulse) * avj

        for index, v, w in zip(self._indices, Vs, Ws):
            velocities[index] = (v, w)

    def _solve_position_constraints(self, step, positions, velocities):
        indices = self._indices
        ratio = self._ratio
        Cs = self._centers
        As = self._angles

        qa, qb, qc, qd = [Mat22(angle=a) for a in As]
        ca, cb, cc, cd = Cs
        aa, ab, ac, ad = As

        ia, ib, ic, id_ = self._inertias
        ma, mb, mc, md = self._masses
        lca, lcb, lcc, lcd = self._local_centers

        mass = 0.0
        if self._revolute_a:
            j_vac = Vec2()
            j_wa = 1.0
            j_wc = 1.0
            mass += ia + ic
            coordinate_a = aa - ac - self._reference_angle_a
        else:
            u = qc * self._local_axis_c
            rc = qc * (self._local_anchor_c - lcc)
            ra = qa * (self._local_anchor_a - lca)
            j_vac = u
            j_wc = rc.cross(u)
            j_wa = ra.cross(u)
            mass += mc + ma + ic * j_wc ** 2 + ia * j_wa ** 2

            pc = self._local_anchor_c - lcc
            pa = qc.mul_t(ra + (ca - cc))
            coordinate_a = (pa - pc).dot(self._local_axis_c)

        if self._revolute_b:
            j_vbd = Vec2()
            j_wb = 1.0
            j_wd = 1.0
            mass += ib + id_

            coordinate_b = ab - ad - self._reference_angle_b
        else:
            u = qd * self._local_axis_d
            rd = qd * (self._local_anchor_d - lcd)
            rb = qb * (self._local_anchor_b - lcb)
            j_vbd = ratio * u
            j_wd = ratio * rd.cross(u)
            j_wb = ratio * rb.cross(u)
            mass += ratio**2 * (md + mb) + id_ * j_wd ** 2 + ib * j_wb ** 2

            pd = self._local_anchor_d - lcd
            pb = qd.mul_t(rb + (cb - cd))
            coordinate_b = (pb - pd).dot(self._local_axis_d)

        C = (coordinate_a + ratio * coordinate_b) - self._constant

        if mass > 0.0:
            impulse = -C / mass

            #Jcs = [j_vac, j_vbd, -j_vac, -j_vbd]
            #Jas = [j_wa, j_wb, -j_wc, -j_wd]
            #for k, (i, m, jc, ja) in enumerate(zip(indices, self._masses, Jcs, Jas)):
            #    c, a = positions[i]
            #    positions[i] = (c + m * impulse * jc, a + i * impulse * ja)

            ca += ma * impulse * j_vac
            cb += mb * impulse * j_vbd
            cc -= mc * impulse * j_vac
            cd -= md * impulse * j_vbd

            aa += ia * impulse * j_wa
            ab += ib * impulse * j_wb
            ac -= ic * impulse * j_wc
            ad -= id_ * impulse * j_wd
            
            for i, c, a in zip(indices, [ca, cb, cc, cd], [aa, ab, ac, ad]):
                positions[i] = (c, a)
    
        return True # linear_error < LINEAR_SLOP # TODO_ERIN not implemented
