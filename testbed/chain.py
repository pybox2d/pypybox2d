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

class Chain (Framework):
    name="Chain"
    def __init__(self):
        super(Chain, self).__init__()

        # The ground
        ground = self.world.create_static_body(shapes=b2.Edge((-40,0),(40,0))) 
        plank=b2.Fixture(
                    shape=b2.Polygon(box=(0.6,0.125)),
                    density=20,
                    friction=0.2,
                )

        # Create one Chain (Only the left end is fixed)
        prev_body = ground
        y = 25
        num_planks = 30
        for i in range(num_planks):
            body = self.world.create_dynamic_body(
                        position=(0.5+i, y), 
                        fixtures=plank,
                    )

            # You can try a weld_joint for a slightly different effect.
            #self.world.create_weld_joint(
            self.world.create_revolute_joint(
                prev_body, body,
                anchor=(i, y),
                )

            prev_body = body

if __name__=="__main__":
     main(Chain)

