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
from random import random

class Breakable (Framework):
    name="Breakable bodies"
    description="With enough of an impulse, the single body will split [press b to manually break it]"
    _break = False # Flag to break
    broke = False  # Already broken?
    def __init__(self):
        super(Breakable, self).__init__()

        # The ground
        ground = self.world.create_static_body()
        ground.create_edge_fixture((-40,0), (40,0))

        # The breakable body
        self.shapes = (b2.Polygon(box=(0.5,0.5,(-0.5,0),0)),
                       b2.Polygon(box=(0.5,0.5,( 0.5,0),0)))

        self.body=self.world.create_dynamic_body(
                    position=(0,40), 
                    angle=0.25 * PI,
                    shapes=self.shapes,
                    shape_fixture=b2.Fixture(density=1),
                )
    
        self.fixtures = self.body.fixtures

        self.world.contact_manager.post_solve = self.post_solve

    def post_solve(self, contact, impulse): 
        # Already broken?
        if self.broke:
            return 
        
        # If the impulse is enough to split the objects, then flag it to break
        max_impulse=max(manifold_point.normal_impulse for manifold_point in contact.manifold.used_points)
        print '%.1g' % max_impulse,
        if max_impulse > 40:
            print('break')
            self._break=True

    def break_(self):
        # Create two bodies from one
        body = self.body
        center = body.world_center

        body.destroy_fixture(self.fixtures[1])
        self.fixture2 = None
        
        body2=self.world.create_dynamic_body(
                position=body.position,
                angle=body.angle,
                shapes=self.shapes[1],
                shape_fixture=b2.Fixture(density=1),
                )
        # Compute consistent velocities for new bodies based on cached velocity.
        velocity1 = self.velocity + scalar_cross(self.angular_velocity, body.world_center - center)
        velocity2 = self.velocity + scalar_cross(self.angular_velocity, body2.world_center - center)

        body.angular_velocity=self.angular_velocity
        body.linear_velocity=velocity1
        body2.angular_velocity=self.angular_velocity
        body2.linear_velocity=velocity2

    def post_step(self):
        if self._break:
            self.break_()
            self.broke=True
            self._break=False
        if not self.broke:
            self.velocity = self.body.linear_velocity
            self.angular_velocity = self.body.angular_velocity

    def key_down(self, key):
         if key == Keys.K_b and not self.broke:
             self._break=True

if __name__=="__main__":
     main(Breakable)

