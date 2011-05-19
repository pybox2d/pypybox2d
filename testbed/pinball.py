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
from math import sqrt

class Pinball (Framework):
    name="Pinball"
    description = '\n'.join(["This tests bullet collision and provides an example of a gameplay scenario.",
                  "Press A to control the flippers."])
    bodies=[]
    joints=[]
    def __init__(self):
        super(Pinball, self).__init__()

        # The ground
        ground = self.world.create_static_body(
                    shapes=b2.Loop((0,-2),(8,6),(8,20),(-8,20),(-8,6)),
                     )

        # Flippers
        p1, p2=(-2, 0), (2, 0)
        flipper = { 'fixtures' : b2.Fixture( shape=b2.Polygon(box=(1.75, 0.1)), density=1) }
        self.leftFlipper=self.world.create_dynamic_body(
                position=p1,
                **flipper
                )
        self.rightFlipper=self.world.create_dynamic_body(
                position=p2,
                **flipper
                )
        
        self.leftJoint=self.world.create_revolute_joint(
            ground, self.leftFlipper,
            local_anchor_a=p1,
            local_anchor_b=(0,0),
            motor_enabled=True,
            limit_enabled=True,
            max_motor_torque=1000,
            motor_speed=0,
            lower_angle=-30.0 * PI / 180.0,
            upper_angle=5.0 * PI / 180.0,
            )
        self.rightJoint=self.world.create_revolute_joint(
            ground, self.rightFlipper,
            motor_speed=0,
            motor_enabled=True,
            limit_enabled=True,
            max_motor_torque=1000,
            local_anchor_a=p2,
            local_anchor_b=(0,0),
            lower_angle=-5.0 * PI / 180.0,
            upper_angle=30.0 * PI / 180.0,
        )


        # Ball
        self.ball=self.world.create_dynamic_body(
            fixtures=b2.Fixture(b2.Circle(radius=0.2), density=1.0),
            bullet=True,
            position=(1,15))

        self.pressed=False

    def key_down(self, key):
        if key==Keys.K_a:
            self.pressed=True

    def key_up(self, key):
        if key==Keys.K_a:
            self.pressed=False

    def pre_step(self):
        if self.pressed:
            self.leftJoint.motor_speed=20
            self.rightJoint.motor_speed=-20
        else:
            self.leftJoint.motor_speed=-10
            self.rightJoint.motor_speed=10

if __name__=="__main__":
     main(Pinball)

