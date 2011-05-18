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

# This test shows how a rope joint can be used to stabilize a chain of
# bodies with a heavy payload. Notice that the rope joint just prevents
# excessive stretching and has no other effect.
# By disabling the rope joint you can see that the Box2D solver has trouble
# supporting heavy bodies with light bodies. Try playing around with the
# densities, time step, and iterations to see how they affect stability.
# This test also shows how to use contact filtering. Filtering is configured
# so that the payload does not collide with the chain."
class Rope (Framework):
    name="Rope Joint Test"
    description="Press j to toggle the rope joint."
    def __init__(self):
        super(Rope, self).__init__()

        # The ground
        self.ground = self.world.create_static_body(shapes=b2.Edge((-40, 0),(40, 0)))

        shape=b2.Polygon(box=(0.5,0.125))

        N=10
        y=15.0
        fixture=b2.Fixture(shape=shape, 
                          friction=0.2, 
                          density=20, 
                          category_bits=0x0001, 
                          mask_bits=(0xFFFF & ~0x0002),
                        )

        prev_body=self.ground
        for i in range(N):
            if i < N-1:
                body = self.world.create_dynamic_body(
                            position=(0.5+i, y), 
                            fixtures=fixture,
                            )
            else:
                shape.set_as_box(1.5, 1.5)
                fixture.density=100
                fixture.category_bits=0x0002
                body = self.world.create_dynamic_body(
                            position=(i, y), 
                            fixtures=fixture,
                            angular_damping=0.4,
                            )
            
            self.world.create_revolute_joint(
                prev_body,
                body,
                anchor=(i, y),
                collide_connected=False, 
                )

            prev_body = body

        extra_length = 0.01
        self._max_length = N - 1.0 + extra_length
        self._anchor_a_y = y
        self.body = body
        self.create_rope()

    def create_rope(self):
        self.rope=self.world.create_rope_joint(
                            self.ground, self.body,
                            max_length=self._max_length,
                            local_anchor_a=(0,self._anchor_a_y), 
                            local_anchor_b=(0,0)
                            )

    def post_step(self):
        if self.rope:
            self.print_('Rope ON')
        else:
            self.print_('Rope OFF')

    def key_down(self, key):
        if key == Keys.K_j:
            if self.rope:
                self.world.destroy_joint(self.rope)
                self.rope=None
            else:
                self.create_rope()

if __name__=="__main__":
     main(Rope)

