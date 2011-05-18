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

from framework import *
from random import random

class Confined (Framework):
    name="Confined space"
    description="Press c to create a circle"
    def __init__(self):
        super(Confined, self).__init__()

        # The ground
        ground = self.world.create_static_body(
                shapes=[ 
                        b2.Edge((-10,  0),( 10,  0)),
                        b2.Edge((-10,  0),(-10, 20)),
                        b2.Edge(( 10,  0),( 10, 20)),
                        b2.Edge((-10, 20),( 10, 20)),
                    ]
                ) 

        # The bodies
        self.radius = radius = 0.5
        column_count=5
        row_count=5

        for j in range(column_count):
            for i in range(row_count):
                self.create_circle((-10 + (2.1 * j + 1 + 0.01 * i) * radius, (2 * i + 1) * radius))

        self.world.gravity = (0,0)
    
    def create_circle(self, pos):
        fixture=b2.Fixture(shape=b2.Circle(self.radius, (0,0)), density=1, friction=0.1) 
        self.world.create_dynamic_body(
                position=pos, 
                fixtures=fixture
                )

    def key_down(self, key):
         if key == Keys.K_c:
            self.create_circle((2.0 * random()-1.0, self.radius * (1.0+random())))

if __name__=="__main__":
     main(Confined)
