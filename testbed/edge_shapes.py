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
from math import cos, sin, pi, sqrt
from random import random

VERTEX_COUNT=80
def get_sinusoid_vertices(x1, vertices):
    y1 = 2.0* cos(x1 / 10.0*pi)
    for i in range(vertices):
        x2 = x1 + 0.5
        y2 = 2.0 * cos(x2/10.0*pi)
        yield (x1,y1), (x2,y2)
        x1, y1 = x2, y2

def get_octagon_vertices(w):
    b = w/(2.0 + sqrt(2.0))
    s = sqrt(2.0) * b
    return [(0.5*s, 0), (0.5 * w, b),
            (0.5 * w, b + s), (0.5 * s, w),
            (-0.5 * s, w), (-0.5 * w, b + s),
            (-0.5 * w, b), (-0.5 * s, 0.0),]

class EdgeShapes (Framework):
    name="Edge Shapes"
    description = "Press 1-5 to drop stuff, and d to delete"
    p1_color=(0.4 * 255, 0.9 * 255, 0.4 * 255)
    s1_color=(0.8 * 255, 0.8 * 255, 0.8 * 255)
    s2_color=(0.9 * 255, 0.9 * 255, 0.4 * 255)
    def __init__(self):
        super(EdgeShapes, self).__init__()

        self.ground=self.world.create_static_body()
        for v1, v2 in get_sinusoid_vertices(-20.0, VERTEX_COUNT):
            self.ground.create_edge_fixture(v1, v2)

        self.shapes=[
            b2.Polygon([(-0.5, 0), (0.5, 0), (0, 1.5)]),
            b2.Polygon([(-0.1, 0), (0.1, 0), (0, 1.5)]),
            b2.Polygon(get_octagon_vertices(1.0)),
            b2.Polygon(box=(0.5,0.5)),
            b2.Circle(0.5),
            ]

        self.angle = pi

    @property
    def bodies(self):
        return [body for body in self.world.bodies 
                    if body != self.ground]
    
    def create_shape(self, shapeindex):
        try:
            shape = self.shapes[shapeindex]
        except IndexError:
            return
        
        pos=(10.0*(2.0*random()-1.0), 10.0*(2.0*random()+1.0)) 

        body = self.world.create_dynamic_body(
                    fixtures=b2.Fixture(shape=shape, friction=0.3),
                    position=pos, 
                    angle=(PI * (2.0*random() - 1.0)),
                )
        if isinstance(shape, b2.Circle):
            body.angular_damping=0.02

    def destroy_body(self):
        if not self.world.locked:
            for body in self.bodies:
                self.world.destroy_body(body)
                break

    def key_down(self, key):
         if key in (Keys.K_1, Keys.K_2, Keys.K_3, Keys.K_4, Keys.K_5):
            self.create_shape(key - Keys.K_1)
         elif key == Keys.K_d:
             self.destroy_body()

    def post_step(self):
        # Set up the raycast line
        length=25.0
        point1=Vec2(0, 10)
        d     =(length * cos(self.angle), length * sin(self.angle))
        point2= point1 + d

        fixture = None
        for lst in self.world.ray_cast(point1, point2):
            fixture, point, normal, fraction = lst
            # lst[-1] = fraction # <-- to update the raycast parameters

        if fixture is not None:
            draw_point(self.screen, point, 0.2, self.p1_color)
            draw_lines(self.screen, [point1, point], self.s1_color)

            head = point + 0.5 * normal
            draw_lines(self.screen, [point, head], self.s2_color)
        else:
            draw_lines(self.screen, [point1, point2], self.s1_color)

        self.angle += 0.25*PI/180

if __name__=="__main__":
     main(EdgeShapes)
