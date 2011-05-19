#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
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

class Bullet (Framework):
    name="Bullet"
    description='A test for very fast moving objects (bullets)'
    def __init__(self):
        super(Bullet, self).__init__()

        self.step_count = 0
        ground = self.world.create_static_body(
                    position=(0,0),
                    shapes=[b2.Edge((-10,0), (10,0)),
                            b2.Polygon(box=(0.2, 1, (0.5, 1), 0))]
                )
        
        self._x=0.20352793
        self.body=self.world.create_dynamic_body(
                    position=(0,4),
                    fixtures=b2.Fixture(shape=b2.Polygon(box=(2, 0.1)), density=1.0),
                )

        self.bullet=self.world.create_dynamic_body(
                    position=(self._x, 10),
                    bullet=True,
                    fixtures=b2.Fixture(shape=b2.Polygon(box=(0.25, 0.25)), density=100.0),
                    linear_velocity=(0,-50)
                )

    def launch(self):
        self.body.transform = [(0,4),0]
        self.body.linear_velocity=(0,0)
        self.body.angular_velocity=0

        self.x=rand_float(-1.0, 1.0)
        self.bullet.transform=[(self.x, 10), 0]
        self.bullet.linear_velocity=(0,-50)
        self.bullet.angular_velocity=0

    def post_step(self):
        self.step_count += 1
        if (self.step_count % 60)==0:
            self.launch()

if __name__=="__main__":
     main(Bullet)
