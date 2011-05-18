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
from math import sqrt, sin, cos


class Raycast (Framework):
    name="Raycast"
    description="Press 1-5 to drop stuff, d to delete, m to switch callback modes"
    p1_color=(0.4 * 255, 0.9 * 255, 0.4 * 255)
    s1_color=(0.8 * 255, 0.8 * 255, 0.8 * 255)
    s2_color=(0.9 * 255, 0.9 * 255, 0.4 * 255)
    def __init__(self):
        super(Raycast, self).__init__()

        self.world.gravity = (0,0)
        # The ground
        ground = self.world.create_static_body(
                    shapes=b2.Edge((-40,0),(40,0))
                )

        # The various shapes
        w = 1.0
        b = w / (2.0 + sqrt(2.0))
        s = sqrt(2.0) * b

        self.shapes = [
                    b2.Polygon([(-0.5,0), (0.5,0), (0,1.5)]),
                    b2.Polygon([(-0.1,0), (0.1,0), (0,1.5)]),
                    b2.Polygon(
                            [ 
                            ( 0.5*s, 0), ( 0.5*w, b), (0.5*w, b + s),
                            ( 0.5*s, w), (-0.5*s, w), (-0.5*w, b + s),
                            (-0.5*w, b), (-0.5*s, 0.0), 
                            ]
                        ),
                    b2.Polygon(box=(0.5, 0.5)),
                    b2.Circle(radius=0.5),
                ]
        self.angle = 0

        self.modes = ['closest', 'any', 'multiple']
        self.mode = self.modes[0]
    
    def create_shape(self, shapeindex):
        try:
            shape = self.shapes[shapeindex]
        except IndexError:
            return
        
        pos=(10.0 * (2.0 * random() - 1.0), 5.0 * (2.0 * random() + 1.0)) 

        body = self.world.create_dynamic_body(
                    fixtures=b2.Fixture(shape=shape, friction=0.3),
                    position=pos, 
                    angle=(PI * (2.0*random() - 1.0)),
                )
        if isinstance(shape, b2.Circle):
            body.angular_damping=0.02

    def destroy_body(self):
        for body in self.world.bodies:
            if not self.world.locked:
                self.world.destroy_body(body)
            break

    def key_down(self, key):
         if key in (Keys.K_1, Keys.K_2, Keys.K_3, Keys.K_4, Keys.K_5):
            self.create_shape(key - Keys.K_1)
         elif key == Keys.K_d:
             self.destroy_body()
         elif key == Keys.K_m:
             idx=(self.modes.index(self.mode) + 1) % len(self.modes)
             self.mode = self.modes[idx]

    def post_step(self):
        def draw_hit(cb_point, cb_normal):
            head = cb_point + 0.5*cb_normal

            draw_point(self.screen, cb_point, 0.2, self.p1_color)
            draw_lines(self.screen, (point1, cb_point), self.s1_color)
            draw_lines(self.screen, (cb_point, head), self.s2_color)

        # Set up the raycast line
        length=11
        point1=Vec2(0, 10)
        d     =(length * cos(self.angle), length * sin(self.angle))
        point2= point1 + d

        mode = self.mode
        points = []
        normals = []
        for lst in self.world.ray_cast(point1, point2):
            fixture, point, normal, fraction = lst

            # You control how the ray proceeds by returning a float (by way of lst[-1]) 
            # that indicates the fractional length of the ray. By returning
            # 0, you set the ray length to zero. By returning the current fraction, you proceed
            # to find the closest point. By returning 1, you continue with the original ray
            # clipping. By returning -1, you will filter out the current fixture (the ray
            # will not hit it).
            points.append(point)
            normals.append(normal)

            if mode == 'closest':
                lst[-1] = fraction
            elif mode == 'any':
                lst[-1] = 0.0
            elif mode == 'multiple':
                lst[-1] = 1.0

        # The callback has been called by this point, and if a fixture was hit it will have been
        # set to callback.fixture.
        self.print_("Mode: %s" % mode)
        if points:
            for point, normal in zip(points, normals):
                draw_hit(point, normal)
            self.print_("Hit")
        else:
            draw_lines(self.screen, (point1, point2), self.s1_color)

        self.angle += 0.25*PI/180
        
if __name__=="__main__":
     main(Raycast)
