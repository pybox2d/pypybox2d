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
from math import ceil, log

class Tiles (Framework):
    name="Tiles"
    description='This stress tests the dynamic tree broad-phase. This also shows that tile based collision\nis _not_ smooth due to Box2D not knowing about adjacency.'
    def __init__(self):
        super(Tiles, self).__init__()

        a=0.5
        def ground_positions():
            N=200
            M=10
            position=Vec2(0,0)
            for i in range(M):
                position.x=-N * a
                for j in range(N):
                    yield position
                    position.x+=2.0*a
                position.y-=2.0*a
            
        ground = self.world.create_static_body(
                    position=(0, -a),
                    shapes=[b2.Polygon(box=(a, a, position, 0)) for position in ground_positions()]
                )

        count=20
        def dynamic_positions():
            x=Vec2(-7.0, 0.75)
            deltaX=(0.5625, 1.25)
            deltaY=(1.125, 0.0)
            for i in range(count):
                y=Vec2(*x)
                for j in range(i, count):
                    yield y
                    y+=deltaY
                x+=deltaX 
        
        for pos in dynamic_positions():
            self.world.create_dynamic_body(
                position=pos,
                fixtures=b2.Fixture(shape=b2.Polygon(box=(a,a)), density=5)
                )

    def post_step(self):
        cm=self.world.contact_manager
        height=cm.broadphase.tree_height
        leaf_count=cm.broadphase.proxy_count
        min_node_count=2 * leaf_count - 1
        min_height=ceil(log(float(min_node_count)) / log(2))
        self.print_('Dynamic tree height=%d, min=%d' % (height, min_height))

if __name__=="__main__":
     main(Tiles)
