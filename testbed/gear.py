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

import framework
from framework import *
from math import sqrt

class Gear (Framework):
    name="Gear"
    description=""
    def __init__(self):
        super(Gear, self).__init__()

        ground=self.world.create_static_body(shapes=b2.Edge((50, 0),(-50, 0)))
        
        circle1 = b2.Circle(1.0)
        circle2 = b2.Circle(2.0)
        box = b2.Polygon(box=(0.5, 0.5))
        
        body1 = self.world.create_dynamic_body(
                    position = (-3.0, 12.0),
                    fixtures = b2.Fixture(circle1, density=5.0),
                )
        
        joint1 = self.world.create_revolute_joint(
                    ground, body1,
                    anchor = body1.position)
        
        body2 = self.world.create_dynamic_body(
                    position = (0.0, 12.0),
                    fixtures = b2.Fixture(circle2, density=5.0),
                )

        joint2 = self.world.create_revolute_joint(
                    ground, body2,
                    anchor = body2.position)

        body3 = self.world.create_dynamic_body(
                    position = (2.5, 12.0),
                    fixtures = b2.Fixture(box, density=5.0),
                )
       
        joint3 = self.world.create_prismatic_joint(
                    ground, body3, 
                    anchor = body3.position, 
                    axis = (0, 1),
                    lower_limit = -5.0,
                    upper_limit = 5.0,
                    limit_enabled = True
                )
                    
        joint4 = self.world.create_gear_joint(
                    joint1, joint2,
                    ratio = circle2.radius / circle1.radius
                )

        joint5 = self.world.create_gear_joint(
                    joint2, joint3,
                    ratio = -1.0 / circle2.radius
                )
        
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5

    def post_step(self):
        ratio = self.joint4.ratio
        value = self.joint1.joint_angle + ratio * self.joint2.joint_angle
        self.print_("theta1 + %4.2f * theta2 = %4.2f" % (ratio, value))

        ratio = self.joint5.ratio
        value = self.joint2.joint_angle + ratio * self.joint3.joint_translation
        self.print_("theta2 + %4.2f * delta = %4.2f" % (ratio, value))

if __name__=="__main__":
     main(Gear)

