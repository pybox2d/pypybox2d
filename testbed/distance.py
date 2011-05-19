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
import framework
from pypybox2d.distance import (shape_distance, DISTANCE_MAX_ITERS)

class Distance (Framework):
    name="Distance"
    description="Use WASD to move and QE to rotate the small rectangle.\nThe distance between the marked points is shown."
    point_a_color=(255,0,0)
    point_b_color=(255,255,0)
    poly_color   =(230,230,230)
    def __init__(self):
        super(Distance, self).__init__()
        # Transform A -- a simple translation/offset of (0,-0.2)
        self.transform_a = Transform()
        self.transform_a.position = (0, -0.2)

        # Transform B -- a translation and a rotation
        self.transform_b = Transform()
        self.position_b = Vec2(12.017401,0.13678508)
        self.angle_b = -0.0109265
        self.transform_b.position = self.position_b
        self.transform_b.angle = self.angle_b

        # The two shapes, transformed by the respective transform[A,B]
        self.polygon_a = b2.Polygon(box=(10,0.2))
        self.polygon_b = b2.Polygon(box=(2,0.1))

        # Zoom in a bit to see the shapes
        framework.PPM = 25
        framework.SCREEN_OFFSETX = 15
        framework.SCREEN_OFFSETY = 10

    def post_step(self):
        # Calculate the distance between the two shapes with the specified transforms
        dist_result=shape_distance(self.polygon_a, self.polygon_b, self.transform_a, self.transform_b, True)
        point_a, point_b, distance, iterations=dist_result

        self.print_('Distance = %g' % distance)
        self.print_('Iterations = %d' % iterations)

        # Manually transform the vertices and draw the shapes
        for shape, transform in [(self.polygon_a, self.transform_a), (self.polygon_b, self.transform_b)]:
            new_verts = [transform*v for v in shape.vertices]
            draw_lines(self.screen, new_verts, self.poly_color)

        draw_point(self.screen, point_a, 0.1, self.point_a_color)
        draw_point(self.screen, point_b, 0.1, self.point_b_color)

    def key_down(self, key):
        if key==Keys.K_a:
            self.position_b -= (0.1, 0)
        elif key==Keys.K_d:
            self.position_b += (0.1, 0)
        elif key==Keys.K_w:
            self.position_b += (0, 0.1)
        elif key==Keys.K_s:
            self.position_b -= (0, 0.1)
        elif key==Keys.K_q:
            self.angle_b += 0.1 * PI
        elif key==Keys.K_e:
            self.angle_b -= 0.1 * PI

        self.transform_b.position = self.position_b
        self.transform_b.angle = self.angle_b

if __name__=="__main__":
     main(Distance)

