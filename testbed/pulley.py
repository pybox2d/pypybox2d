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

class Pulley (Framework):
    name="Pulley"
    def __init__(self):
        super(Pulley, self).__init__()
        y, L, a, b=16.0, 12.0, 1.0, 2.0
        # The ground
        ground=self.world.create_static_body(
                    shapes=[
                            b2.Edge((-40,0),(40,0)),
                            b2.Circle(2, (-10.0,y+b+L)),
                            b2.Circle(2, ( 10.0,y+b+L)),
                        ]
                )

        body_a = self.world.create_dynamic_body(
                    position=(-10,y),
                    fixtures=b2.Fixture(shape=b2.Polygon(box=(a,b)), density=5.0),
                    )
        body_b = self.world.create_dynamic_body(
                    position=( 10,y),
                    fixtures=b2.Fixture(shape=b2.Polygon(box=(a,b)), density=5.0),
                    )

        self.pulley=self.world.create_pulley_joint(
                body_a, body_b,
                anchor_a=(-10.0, y+b),
                anchor_b=( 10.0, y+b),
                ground_anchor_a=(-10.0, y+b+L),
                ground_anchor_b=( 10.0, y+b+L),
                ratio=1.5,
            )


    def post_step(self):
        ratio=self.pulley.ratio
        L=self.pulley.length_a + self.pulley.length_b*ratio
        self.print_('L1 + %4.2f * L2 = %4.2f' % (ratio, L))

if __name__=="__main__":
     main(Pulley)

