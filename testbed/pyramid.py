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
from copy import copy

class Pyramid (Framework):
    name="Pyramid"
    def __init__(self):
        super(Pyramid, self).__init__()
        # The ground
        self.world.create_static_body(shapes=b2.Edge((-40, 0), (40, 0)))

        box_half_size = (0.5, 0.5)
        box_density = 5.0
        box_rows = 20

        x=Vec2(-7, 0.75)
        delta_x=(0.5625, 1.25)
        delta_y=(1.125, 0)

        fixture = b2.Fixture(b2.Polygon(box=box_half_size), density=box_density)
        for i in range(box_rows):
            y = copy(x)

            for j in range(i, box_rows):
                self.world.create_dynamic_body(position=y, fixtures=fixture)
                y += delta_y

            x += delta_x

if __name__=="__main__":
     main(Pyramid)
