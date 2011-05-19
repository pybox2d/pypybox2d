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

class CollisionProcessing (Framework):
    name="CollisionProcessing"
    def pre_solve(self, contact, old_manifold):
        manifold = contact.manifold

        if manifold.point_count == 0:
            return

        state1, state2 = old_manifold.get_point_states(manifold)
        if not state2:
            return

        world_manifold = contact.world_manifold
        for point, state in zip(world_manifold.points, state2):
            self.points.append(
                {
                'fixture_a' : contact.fixture_a,
                'fixture_b' : contact.fixture_b,
                'position' : point,
                'normal' : world_manifold.normal,
                'state' : state
                }
            )
                

    def __init__(self):
        super(CollisionProcessing, self).__init__()
        
        self.points = []
        self.world.contact_manager.pre_solve = self.pre_solve

        # Ground body
        world=self.world
        ground = world.create_static_body(shapes=b2.Edge((-50,0),(50,0)))

        xlow, xhi = -5, 5
        ylow, yhi = 2, 35
        random_vector = lambda: Vec2(rand_float(xlow, xhi), rand_float(ylow, yhi))

        # Small triangle
        triangle=b2.Fixture(
                shape=b2.Polygon([(-1,0),(1,0),(0,2)]),
                density=1,
                )

        world.create_dynamic_body(
                position=random_vector(),
                fixtures=triangle,
                user_data='small triangle'
                )

        # Large triangle (recycle definitions)
        triangle.shape.vertices = [2.0 * v for v in triangle.shape.vertices]

        world.create_dynamic_body(
                position=random_vector(),
                fixtures=triangle,
                fixed_rotation=True, # <-- note that the large triangle will not rotate
                user_data='large triangle'
                )

        # Small box
        box=b2.Fixture(
                shape=b2.Polygon(box=(1, 0.5)),
                density=1,
                restitution=0.1,
                )

        world.create_dynamic_body(
                position=random_vector(),
                fixtures=box,
                user_data='small box'
                )

        # Large box
        box.shape.set_as_box(2, 1)
        world.create_dynamic_body(
                position=random_vector(),
                fixtures=box,
                user_data='large box'
            )

        # Small circle
        circle=b2.Fixture(
                shape=b2.Circle(radius=1),
                density=1,
                )

        world.create_dynamic_body(
                    position=random_vector(),
                    fixtures=circle,
                    user_data='small circle'
                )

        # Large circle
        circle.shape.radius *= 2
        world.create_dynamic_body(
                    position=random_vector(),
                    fixtures=circle,
                    user_data='large circle'
                )

    def pre_step(self):
        # We are going to destroy some bodies according to contact
        # points. We must buffer the bodies that should be destroyed
        # because they may belong to multiple contact points.
        nuke = set()
        
        # Traverse the contact results. Destroy bodies that
        # are touching heavier bodies.
        body_pairs = [(p['fixture_a'].body, p['fixture_b'].body) for p in self.points]
        
        for body1, body2 in body_pairs:
            mass1, mass2 = body1.mass, body2.mass
        
            if mass1 > 0.0 and mass2 > 0.0:
                if mass2 > mass1:
                    nuke_body = body1
                else:
                    nuke_body = body2
        
                nuke.add(nuke_body)
        
        # Destroy the bodies, skipping duplicates.
        for b in nuke:
            print("Nuking:", b.user_data)
            self.world.destroy_body(b)
        
        nuke = None

        self.points = []

if __name__=="__main__":
     main(CollisionProcessing)
