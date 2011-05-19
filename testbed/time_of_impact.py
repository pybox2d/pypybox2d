#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 Ken Lauer / sirkne at gmail dot com
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

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from framework import *
from math import sqrt
import pypybox2d.toi as TOI
from pypybox2d.toi import time_of_impact

class TimeOfImpact (Framework):
    name="Time of Impact"
    description="See the source code for more information. No additional controls."
    def __init__(self):
        super(TimeOfImpact, self).__init__()

        # The two shapes to check for time of impact
        self.shape_a = b2.Polygon(box=(0.2, 1, (0.5, 1), 0))
        self.shape_b = b2.Polygon(box=(2, 0.1))

    def post_step(self):
        # Sweep describes the motion of a body/shape for TOI computation.
        # Shapes are defined with respect to the body origin, which may
        # no coincide with the center of mass. However, to support dynamics
        # we must interpolate the center of mass position.
        sweep_a=Sweep(c0=(0,0), c=(0,0),
                       a=0, a0=0,
                       local_center=(0,0))

        # The parameters of the sweep are defined as follows:
        # local_center- local center of mass position
        # c0, c       - center world positions
        # a0, a       - world angles
        # t0          - time interval = [t0,1], where t0 is in [0,1]

        sweep_b = Sweep(c0=(-0.20382018, 2.1368704),
                         a0=-3.1664171,
                         c=(-0.26699525, 2.3552670),
                         a=-3.3926492,
                         local_center=(0,0))

        type_, toi = time_of_impact(self.shape_a, self.shape_b, sweep_a, sweep_b, 1.0)

        self.print_("TOI = %g" % toi)
        self.print_("max toi iters = %d, max root iters = %d" % (TOI.toi_max_iters, TOI.toi_max_root_iters))

        # Draw the shapes at their current position (t=0)
        # shape_a (the vertical polygon)
        transform = sweep_a.get_transform(0)
        draw_unattached_polygon(self.shape_a, self.screen, transform, (0.9*255, 0.9*255, 0.9*255))

        # shape_b (the horizontal polygon)
        transform = sweep_b.get_transform(0)
        draw_unattached_polygon(self.shape_b, self.screen, transform, (0.5*255, 0.9*255, 0.5*255))

        # local_point=(2, -0.1)
        # r_b = transform * local_point - sweep_b.c0
        # w_b = sweep_b.a - sweep_b.a0
        # v_b = sweep_b.c - sweep_b.c0
        # v = v_b + scalar_cross(w_b, r_b)

        # Now, draw shape_b in a different color when they would collide (i.e., at t=time of impact)
        # This shows that the polygon would rotate upon collision
        transform = sweep_b.get_transform(toi)
        draw_unattached_polygon(self.shape_b, self.screen, transform, (0.5*255, 0.7*255, 0.9*255))

        # And finally, draw shape_b at t=1.0, where it would be if it did not collide with shape_a
        # In this case, time_of_impact = 1.0, so these become the same polygon.
        transform = sweep_b.get_transform(1.0)
        draw_unattached_polygon(self.shape_b, self.screen, transform, (0.9*255, 0.5*255, 0.5*255))

if __name__=="__main__":
     main(TimeOfImpact)
