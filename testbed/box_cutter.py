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

# Original C++ version by Daid 
#  http://www.box2d.org/forum/viewtopic.php?f=3&t=1473
# - Written for pybox2d 2.1 by Ken

from framework import *
from math import sin, cos, pi
import sys

LASER_HALF_WIDTH=2
LASER_SPLIT_SIZE=0.1
LASER_SPLIT_TAG='can_cut'

def _polygon_split(fixture, p1, p2, split_size):
    polygon=fixture.shape
    body=fixture.body
    transform=body.transform

    local_entry=body.get_local_point(p1)
    local_exit=body.get_local_point(p2)
    entry_vector=local_exit-local_entry
    entry_normal=entry_vector.cross(1.0)
    last_verts=None
    new_vertices=[[], []]
    cut_added=[-1,-1]
    for vertex in polygon.vertices:
        # Find out if this vertex is on the new or old shape
        if entry_normal.dot(Vec2(*vertex) - local_entry) > 0.0:
            verts=new_vertices[0]
        else:
            verts=new_vertices[1]

        if last_verts!=verts:
            # if we switch from one shape to the other, add the cut vertices
            if last_verts==new_vertices[0]:
                if cut_added[0]!=-1:
                    return []
                cut_added[0]=len(last_verts)
                last_verts.append(Vec2(*local_exit))
                last_verts.append(Vec2(*local_entry))
            elif last_verts==new_vertices[1]:
                if cut_added[1]!=-1:
                    return []
                cut_added[1]=len(last_verts)
                last_verts.append(Vec2(*local_entry))
                last_verts.append(Vec2(*local_exit))

        verts.append(Vec2(*vertex))
        last_verts=verts

    # Add the cut if not added yet
    if cut_added[0] < 0:
        cut_added[0]=len(new_vertices[0])
        new_vertices[0].append(Vec2(*local_exit))
        new_vertices[0].append(Vec2(*local_entry))
    if cut_added[1] < 0:
        cut_added[1]=len(new_vertices[1])
        new_vertices[1].append(Vec2(*local_entry))
        new_vertices[1].append(Vec2(*local_exit))

    # Cut based on the split size
    for added, verts in zip(cut_added, new_vertices):
        if added > 0:
            offset=verts[added-1]-verts[added]
        else:
            offset=verts[-1]-verts[0]
        offset.normalize()
        verts[added]+=split_size*offset

        if added < len(verts)-2:
            offset=verts[added+2]-verts[added+1]
        else:
            offset=verts[0]-verts[len(verts)-1]
        offset.normalize()
        verts[added+1]+=split_size*offset

    # Ensure the new shapes aren't too small
    for verts in new_vertices:
        for i, v1 in enumerate(verts):
            for j, v2 in enumerate(verts):
                if i!=j and (v1-v2).length < 0.1:
                    # print('Failed to split: too small')
                    return []

    try:
        return [b2.Polygon(vertices=verts) for verts in new_vertices]
    except b2AssertException:
        return []
    except ValueError:
        return []

def _laser_cut(world, laser_body, length=30.0, laser_half_width=2, **kwargs):
    def get_hits(_p1, _p2):
        hits = []
        for lst in world.ray_cast(_p1, _p2):
            fixture, point, normal, fraction = lst
            
            if fixture.body.user_data == LASER_SPLIT_TAG:
                hits.append((fixture,point))

            lst[-1] = 1.0 # notify the ray cast to continue
        return hits

    p1, p2=get_laser_line(laser_body, length, laser_half_width)

    hits_forward = get_hits(p1, p2)
    if not hits_forward:
        return []
    
    hits_reverse = get_hits(p2, p1)
    if len(hits_forward) != len(hits_reverse):
        return []

    ret=[]
    for (fixture1, point1), (fixture2, point2) in zip(hits_forward, hits_reverse):
        if fixture1 != fixture2:
            continue
        
        new_polygons=_polygon_split(fixture1, point1, point2, LASER_SPLIT_SIZE)
        if new_polygons:
            ret.append((fixture1, new_polygons))

    return ret

def laser_cut(world, laser_body, length=30.0, laser_half_width=2, **kwargs):
    cut_fixtures=_laser_cut(world, laser_body, laser_half_width=LASER_HALF_WIDTH)
    remove_bodies=[]
    for fixture, new_shapes in cut_fixtures:
        body=fixture.body
        if body in remove_bodies:
            continue

        new_body=world.create_dynamic_body(
                user_data=LASER_SPLIT_TAG,
                position=body.position,
                angle=body.angle,
                linear_velocity=body.linear_velocity,
                angular_velocity=body.angular_velocity,
                )

        try:
            new_body.create_fixture(
                        new_shapes[1],
                        friction=fixture.friction,
                        restitution=fixture.restitution,
                        density=fixture.density,
                    )
        except AssertionError:
            print('New body fixture failed: %s' % sys.exc_info()[1])
            remove_bodies.append(new_body)

        try:
            new_fixture=body.create_fixture(
                        new_shapes[0],
                        friction=fixture.friction,
                        restitution=fixture.restitution,
                        density=fixture.density,
                    )

            body.destroy_fixture(fixture)
        except AssertionError:
            print('New fixture/destroy failed: %s' % sys.exc_info()[1])
            remove_bodies.append(body)

    for body in remove_bodies:
        world.destroy_body(body)

def get_laser_line(laser_body, length, laser_half_width):
    laser_start=(laser_half_width-0.1, 0.0)
    laser_dir  =(length, 0.0)
    p1 = laser_body.get_world_point(laser_start)
    p2 = p1 + laser_body.get_world_vector(laser_dir)
    return (p1, p2)

def laser_display(screen, laser_body, length=30.0, laser_color=(255,0,0), laser_half_width=2, **kwargs):
    p1, p2=get_laser_line(laser_body, length, laser_half_width)
    draw_line(screen, p1, p2, laser_color)

class BoxCutter(Framework):
    name="Box Cutter"
    description='Press (c) to cut'
    move=0
    jump=100
    def __init__(self):
        super(BoxCutter, self).__init__()
        # The ground
        self.ground=self.world.create_static_body(
                user_data='ground',
                shapes=[
                        b2.Edge((-50,0),( 50, 0)),
                        b2.Edge((-50,0),(-50,10)),
                        b2.Edge(( 50,0),( 50,10)),
                    ]
        )

        self.laser_body=self.world.create_dynamic_body(
                user_data='laser',
                position=(0,2))

        self.laser_body.create_polygon_fixture(box=(LASER_HALF_WIDTH, 1), density=4.0)

        for i in range(2):
            body = self.world.create_dynamic_body(
                    user_data=LASER_SPLIT_TAG,
                    position=(3.0+i*6,8),
                    )
            body.create_polygon_fixture(box=(3.0, 3.0), density=5.0)

    def key_down(self, key):
        if key==Keys.K_c:
            laser_cut(self.world, self.laser_body, laser_half_width=LASER_HALF_WIDTH)

    def post_step(self):
        laser_display(self.screen, self.laser_body, laser_half_width=LASER_HALF_WIDTH)

if __name__=="__main__":
     main(BoxCutter)
