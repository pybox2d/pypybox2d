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

class FrictionJointTest(Framework):
    name="FrictionJoint example"
    description="""From left to right, the polygons have an increasing
coefficient of FrictionJoint"""
    def __init__(self):
        Framework.__init__(self)

        # The ground
        self.world.create_static_body(shapes=b2.Edge((-20, 10), (20, 20))) 

        hw, hh = 0.5, 0.5 # half width, half height

        # The bodies
        for i, friction in enumerate([0.0, 0.1, 0.35, 0.5, 0.75, 0.99]):
            self.world.create_dynamic_body(
                    position=(-10+3.0*i, 20), 
                    fixtures=b2.Fixture(shape=b2.Polygon(box=(hw, hh)), 
                                        density=1.0, friction=friction)
                                        )

if __name__=="__main__":
     main(FrictionJointTest)
