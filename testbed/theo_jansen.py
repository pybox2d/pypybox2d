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

# Original inspired by a contribution by roman_m
# Dimensions scooped from APE (http://www.cove.org/ape/index.htm)
class TheoJansen (Framework):
    name="Theo Jansen"
    description="Keys: left = a, brake = s, right = d, toggle motor = m"
    motor_speed = 2
    motorOn = True
    offset = (0, 8)

    def __init__(self):
        super(TheoJansen, self).__init__()
        
        # 
        ball_count=40
        pivot = Vec2(0, 0.8)

        # The ground
        ground = self.world.create_static_body(
                shapes=[ 
                        b2.Edge((-50,0),(50,0)),
                        b2.Edge((-50,0),(-50,10)),
                        b2.Edge((50,0),(50,10)),
                    ]
                ) 

        box=b2.Fixture(
                shape=b2.Polygon(box=(0.5,0.5)),
                density=1,
                friction=0.3)
        circle=b2.Fixture(
                shape=b2.Circle(radius=0.25),
                density=1)

        # create the balls on the ground
        for i in range(ball_count):
            self.world.create_dynamic_body(
                    fixtures=circle,
                    position=(-40+2.0*i,0.5),
                    )

        # The chassis
        chassis_fixture=b2.Fixture(
                shape=b2.Polygon(box=(2.5,1)),
                density=1,
                friction=0.3,
                group_index=-1)

        self.chassis = self.world.create_dynamic_body(
                fixtures=chassis_fixture,
                position=pivot + self.offset)

        # Chassis wheel
        wheel_fixture=b2.Fixture(
                shape=b2.Circle(radius=1.6),
                density=1,
                friction=0.3,
                group_index=-1)

        self.wheel = self.world.create_dynamic_body(
                fixtures=wheel_fixture,
                position=pivot + self.offset)

        # Add a joint between the chassis wheel and the chassis itself
        self.motor_joint=self.world.create_revolute_joint(
                self.wheel, self.chassis, 
                anchor=pivot+self.offset,
                collide_connected=False,
                motor_speed=self.motor_speed,
                max_motor_torque=400,
                motor_enabled=self.motorOn)

        wheel_anchor = pivot + (0, -0.8)
        self.create_leg(-1, wheel_anchor)
        self.create_leg( 1, wheel_anchor)

        self.wheel.transform = (self.wheel.position, 120.0 * PI / 180)
        self.create_leg(-1, wheel_anchor)
        self.create_leg( 1, wheel_anchor)

        self.wheel.transform = (self.wheel.position,-120.0 * PI / 180)
        self.create_leg(-1, wheel_anchor)
        self.create_leg( 1, wheel_anchor)

    def create_leg(self, s, wheel_anchor):
        p1, p2=Vec2(5.4*s,-6.1), Vec2(7.2*s,-1.2)
        p3, p4=Vec2(4.3*s,-1.9), Vec2(3.1*s, 0.8)
        p5, p6=Vec2(6.0*s, 1.5), Vec2(2.5*s, 3.7)

        # Use a simple system to create mirrored vertices
        if s > 0:
            poly1=b2.Polygon(vertices=(p1, p2, p3))
            poly2=b2.Polygon(vertices=((0,0),p5-p4,p6-p4))
        else:
            poly1=b2.Polygon(vertices=(p1, p3, p2))
            poly2=b2.Polygon(vertices=((0,0),p6-p4,p5-p4))

        body1=self.world.create_dynamic_body(
                position=self.offset,
                angular_damping=10,
                fixtures=b2.Fixture(
                    shape=poly1,
                    group_index=-1,
                    density=1),
                )

        body2=self.world.create_dynamic_body(
                position=p4 + self.offset,
                angular_damping=10,
                fixtures=b2.Fixture(
                    shape=poly2,
                    group_index=-1,
                    density=1),
                )

        # Using a soft distance constraint can reduce some jitter.
        # It also makes the structure seem a bit more fluid by
        # acting like a suspension system.
        # Now, join all of the bodies together with distance joints,
        # and one single revolute joint on the chassis
        self.world.create_distance_joint(
                body1, body2,
                damping_ratio=0.5,
                frequency=10,
                anchor_a=p2+self.offset,
                anchor_b=p5+self.offset,
                )

        self.world.create_distance_joint(
                body1, body2,
                damping_ratio=0.5,
                frequency=10,
                anchor_a=p3+self.offset,
                anchor_b=p4+self.offset,
                )

        self.world.create_distance_joint(
                body1, self.wheel,
                damping_ratio=0.5,
                frequency=10,
                anchor_a=p3+self.offset,
                anchor_b=wheel_anchor+self.offset,
                )
        
        self.world.create_distance_joint(
                body2, self.wheel,
                damping_ratio=0.5,
                frequency=10,
                anchor_a=p6+self.offset,
                anchor_b=wheel_anchor+self.offset,
                )

        self.world.create_revolute_joint(
                body2, self.chassis,
                anchor=p4+self.offset,
                )

    def key_down(self, key):
        if key==Keys.K_a:
            self.motor_joint.motor_speed=-self.motor_speed
        elif key==Keys.K_d:
            self.motor_joint.motor_speed=self.motor_speed
        elif key==Keys.K_s:
            self.motor_joint.motor_speed=0
        elif key==Keys.K_m:
            self.motor_joint.motor_enabled=not self.motor_joint.motor_enabled

if __name__=="__main__":
     main(TheoJansen)

