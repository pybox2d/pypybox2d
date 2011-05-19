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

def create_bridge(world, ground, size, offset, plank_count, friction=0.6, density=1.0):
    """
    Create a bridge with plank_count planks,
    utilizing rectangular planks of size (width, height).
    The bridge should start at x_offset, and continue to
    roughly x_offset+width*plank_count.
    The y will not change.
    """
    width, height=size
    x_offset, y_offset=offset
    plank=b2.Fixture( 
                shape=b2.Polygon(box=(width/2,height/2)),
                friction=friction,
                density=density,
                )
   
    bodies=[] 
    prev_body = ground
    for i in range(plank_count):
        body = world.create_dynamic_body(
                    position=(x_offset+width*i, y_offset),
                    fixtures=plank,
                )
        bodies.append(body)

        world.create_revolute_joint(
                prev_body,
                body,
                anchor=(x_offset+width*(i-0.5),y_offset)
            )
        
        prev_body = body

    world.create_revolute_joint(
            prev_body,
            ground,
            anchor=(x_offset+width*(plank_count-0.5),y_offset),
        )
    return bodies

class Bridge (Framework):
    name="Bridge"
    num_planks = 30 # Number of planks in the bridge
    def __init__(self):
        super(Bridge, self).__init__()

        # The ground
        ground = self.world.create_static_body(shapes=b2.Edge((-40,0),(40,0)))

        create_bridge(self.world, ground, (1.0,0.25), (-14.5,5), self.num_planks, 0.2, 20)

        fixture=b2.Fixture(
                shape=b2.Polygon([(-0.5,0.0), ( 0.5,0.0), ( 0.0,1.5), ]),
                    density=1.0 ) 
        for i in range(2):
            self.world.create_dynamic_body(
                    position=(-8+8*i,12), 
                    fixtures=fixture,
                    )

        fixture=b2.Fixture(shape=b2.Circle(radius=0.5), density=1)
        for i in range(3):
            self.world.create_dynamic_body(
                    position=(-6+6*i,10),
                    fixtures=fixture,
                    )

if __name__=="__main__":
     main(Bridge)

