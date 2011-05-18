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

class CollisionFiltering (Framework):
    name="Collision Filtering"
    description="See which shapes collide with each other."
    # This is a test of collision filtering.
    # There is a triangle, a box, and a circle.
    # There are 6 shapes. 3 large and 3 small.
    # The 3 small ones always collide.
    # The 3 large ones never collide.
    # The boxes don't collide with triangles (except if both are small).
    # The box connected to the large triangle has no filter settings,
    # so it collides with everything.
    def __init__(self):
        super(CollisionFiltering, self).__init__()
        # Ground body
        world = self.world
        ground = world.create_static_body(shapes=b2.Edge((-40,0),(40,0)))

        # Define the groups that fixtures can fall into
        # Note that negative groups never collide with other negative ones.
        small_group = 1
        large_group = -1

        # And the categories
        # Note that these are bit-locations, and as such are written in hexadecimal.
        default_category = 0x0001 
        triangle_category = 0x0002
        box_category = 0x0004
        circle_category = 0x0008

        # And the masks that define which can hit one another
        # A mask of 0xFFFF means that it will collide with everything
        # else in its group. The box mask below uses an exclusive OR (XOR)
        # which in effect toggles the triangleCategory bit, making 
        # boxMask = 0xFFFD. Such a mask means that boxes never collide with triangles.
        # (if you're still confused, see the implementation details below)

        triangle_mask = 0xFFFF
        box_mask = 0xFFFF ^ triangle_category 
        circle_mask = 0xFFFF

        # The actual implementation determining whether or not two objects collide is 
        # defined in the library source code, but it can be overridden in Python (with world.contact_filter).
        # The default behavior goes like this:
        #   if (filter_a.group_index == filter_b.group_index and filter_a.group_index != 0):
        #       collide if filter_a.group_index is greater than zero (negative groups never collide)
        #   else:
        #       collide if (filter_a.mask_bits & filter_b.category_bits) != 0 and (filter_a.category_bits & filter_b.mask_bits) != 0
        #
        # So, if they have the same group index (and that index isn't the default 0),
        # then they collide if the group index is > 0 (since negative groups never collide)
        # (Note that a body with the default filter settings will always collide with everything
        # else.)
        # If their group indices differ, then only if their bitwise-ANDed category and mask bits match
        # up do they collide. 
        # For more help, some basics of bit masks might help:
        # -> http://en.wikipedia.org/wiki/Mask_%28computing%29
        
        # Small triangle
        triangle=b2.Fixture(
                shape=b2.Polygon([(-1,0),(1,0),(0,2)]),
                density=1,
                group_index = small_group,
                category_bits = triangle_category,
                mask_bits = triangle_mask,
                )

        world.create_dynamic_body(
                position=(-5,2),
                fixtures=triangle,
                )

        # Large triangle (recycle definitions)
        triangle.shape.vertices = [2.0*v for v in triangle.shape.vertices]
        triangle.group_index = large_group

        trianglebody=world.create_dynamic_body(
                position=(-5,6),
                fixtures=triangle,
                fixed_rotation=True, # <-- note that the large triangle will not rotate
                )

        # Small box
        box=b2.Fixture(
                shape=b2.Polygon(box=(1, 0.5)),
                density=1,
                restitution=0.1,
                group_index = small_group,
                category_bits = box_category,
                mask_bits = box_mask,
                )

        world.create_dynamic_body(
                position=(0,2),
                fixtures=box,
                )

        # Large box
        box.shape.set_as_box(2, 1)
        box.group_index = large_group
        world.create_dynamic_body(
                position=(0,6),
                fixtures=box,
            )

        # Small circle
        circle=b2.Fixture(
                shape=b2.Circle(radius=1),
                density=1,
                group_index = small_group,
                category_bits = circle_category,
                mask_bits = circle_mask,
                )

        world.create_dynamic_body(
                    position=(5,2),
                    fixtures=circle,
                )

        # Large circle
        circle.shape.radius *= 2
        circle.group_index = large_group
        world.create_dynamic_body(
                    position=(5,6),
                    fixtures=circle,
                )

        # Create a joint for fun on the big triangle
        # Note that it does not inherit or have anything to do with the
        # filter settings of the attached triangle. 
        box=b2.Fixture(shape=b2.Polygon(box=(0.5, 1)), density=1)

        testbody=world.create_dynamic_body(
                    position=(-5,10),
                    fixtures=box,
                )
        world.create_prismatic_joint(
            trianglebody,
            testbody,
            limit_enabled=True,
            local_anchor_a=(0, 4),
            local_anchor_b=(0, 0),
            local_x_axis=(0, 1),
            lower_limit=-1,
            upper_limit=1,
            )


if __name__=="__main__":
     main(CollisionFiltering)
