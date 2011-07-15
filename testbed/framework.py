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

"""
A simple, minimal Pygame-based backend for pypybox2d.

It will only draw and support very basic keyboard input (ESC to quit). 
There is also basic mouse joint support.
"""

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

import pygame
from pygame.locals import *
import sys
sys.path.extend(['..', '.'])

import pypybox2d as b2
from pypybox2d.common import *
import pypybox2d.shapes as shapes
from random import uniform as rand_float

TARGET_FPS=60
PPM=10.0
TIMESTEP=1.0/TARGET_FPS
VEL_ITERS, POS_ITERS=8,3
SCREEN_WIDTH, SCREEN_HEIGHT=800, 600
SCREEN_OFFSETX, SCREEN_OFFSETY=30.0, 40.0
colors = {
    b2.Body.STATIC    : (255,255,255,255),
    b2.Body.DYNAMIC   : (127,127,127,255),
    b2.Body.KINEMATIC : (127,127,230,255),
}

def to_screen(vertices):
    return [(int((SCREEN_OFFSETX+v[0])*PPM), int((SCREEN_OFFSETY-v[1])*PPM)) for v in vertices]
def to_world(vertices):
    return [(float(v[0]) / PPM - SCREEN_OFFSETX, SCREEN_OFFSETY - float(v[1]) / PPM) for v in vertices]

#screenx = (screen_off_x + v0) * ppm
#screenx / ppm - screen_off_x

#screeny = (screen_off_y - v1) * ppm
#screen_off_y - screeny / ppm = v1

def draw_unattached_polygon(polygon, screen, transform, color):
    vertices = to_screen([transform*v for v in polygon.vertices])
    pygame.draw.polygon(screen, [c/2.0 for c in color], vertices, 0)
    pygame.draw.polygon(screen, color, vertices, 1)

def draw_polygon(polygon, screen, fixture, body=None, transform=None, color=None):
    if transform is None:
        transform = body.transform
    if color is None:
        color = colors[body.type]
    vertices = to_screen([transform*v for v in polygon.vertices])
    pygame.draw.polygon(screen, [c/2.0 for c in color], vertices, 0)
    pygame.draw.polygon(screen, color, vertices, 1)
b2.Polygon.draw = draw_polygon

def draw_circle(circle, screen, fixture, body=None, transform=None, color=None):
    if transform is None:
        transform = body.transform
    if color is None:
        color = colors[body.type]
    radius = circle.radius * PPM
    axis = transform.rotation.col1
    c = to_screen([transform*circle.position])[0]
    pygame.draw.circle(screen, colors[body.type], c, int(radius))
    pygame.draw.aaline(screen, (255,255,255), c, (int(c[0] - radius*axis[0]), int(c[1] + radius*axis[1])))
b2.Circle.draw = draw_circle

def draw_edge(edge, screen, fixture, body=None, transform=None, color=None):
    if transform is not None:
        transform = body.transform
    if color is None:
        color = colors[body.type]
    vertices = to_screen([transform*edge.vertex1, transform*edge.vertex2])
    pygame.draw.line(screen, colors[body.type], vertices[0], vertices[1])
b2.Edge.draw = draw_edge

def draw_loop(loop, screen, fixture, body=None, transform=None, color=None):
    if transform is None:
        transform = body.transform
    if color is None:
        color = colors[body.type]
    vertices = to_screen([transform*v for v in loop.vertices])
    v1 = vertices[-1]
    for v2 in vertices:
        pygame.draw.line(screen, colors[body.type], v1, v2)
        v1 = v2
b2.Loop.draw = draw_loop

def draw_line(screen, p1, p2, color=(255, 255, 255)):
    p1, p2 = to_screen([p1, p2])
    pygame.draw.line(screen, color, p1, p2)

def draw_point(screen, point, radius=0.1, color=(255, 255, 255)):
    point = to_screen([point])[0]
    pygame.draw.circle(screen, color, point, radius * PPM)

def draw_lines(screen, points, color=(255, 255, 255)):
    vertices = to_screen(points)
    v1 = vertices[-1]
    for v2 in vertices:
        pygame.draw.line(screen, color, v1, v2)
        v1 = v2

def draw_world(screen, world):
    """Draw the world"""
    for body in world.bodies:
        transform = body.transform
        for fixture in body.fixtures:
            fixture.shape.draw(screen, fixture, body, transform)

class Keys(object):
    pass

# Set up the keys (needed as the normal framework abstracts them between backends)
keys = [s for s in dir(pygame.locals) if s.startswith('K_')]
for key in keys:
    value=getattr(pygame.locals, key)
    setattr(Keys, key, value)

class Framework(object):
    name='None'
    description=''
    def __init__(self):
        self.world=b2.World()

        self.world.fixture_destruction_listener = self.fixture_destroyed
        self.world.joint_destruction_listener = self.joint_destroyed

        print('Initializing pygame framework...')
        # Pygame Initialization
        pygame.init()
        caption= "Pure Python Box2D Testbed - " + self.name
        pygame.display.set_caption(caption)

        # Screen and debug draw
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.Font(None, 15)

        self.groundbody = self.world.create_static_body()
        self.mouse_joint = None

    def run(self):
        """
        Main loop.

        Updates the world and then the screen.
        """

        running = True
        clock = pygame.time.Clock()
        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == Keys.K_ESCAPE):
                    running=False
                elif event.type == KEYDOWN:
                    self.key_down(event.key)
                elif event.type == KEYUP:
                    self.key_up(event.key)
                elif event.type in (MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION):
                    p = Vec2(*to_world([event.pos])[0])
                    if event.type == MOUSEBUTTONDOWN:
                        self.mouse_down(p)
                    elif event.type == MOUSEBUTTONUP:
                        self.mouse_up(p)
                    elif event.type == MOUSEMOTION:
                        self.mouse_move(p)
  
            self.screen.fill((0, 0, 0))

            self.text_line=15

            self.pre_step()

            # Step the world
            self.world.step(TIMESTEP, VEL_ITERS, POS_ITERS)
            self.world.clear_forces()

            draw_world(self.screen, self.world)

            self.post_step()

            # Draw the name of the test running
            self.print_(self.name, (127,127,255))

            if self.description:
                # Draw the name of the test running
                for s in self.description.split('\n'):
                    self.print_(s, (127,255,127))

            pygame.display.flip()
            clock.tick(TARGET_FPS)
            self.fps = clock.get_fps()

    def print_(self, str, color=(229,153,153,255)):
        """
        Draw some text at the top status lines
        and advance to the next line.
        """
        self.screen.blit(self.font.render(str, True, color), (5,self.text_line))
        self.text_line += 15

    def mouse_down(self, p):
        if self.mouse_joint:
            return
        
        # Create a mouse joint on the selected body (assuming it's dynamic)
        # Make a small box.
        aabb = AABB(lower_bound=p - (0.001, 0.001), upper_bound=p + (0.001, 0.001))
        body = None

        # Query the world for overlapping shapes.
        for fixture in self.world.query_aabb(aabb):
            body = fixture.body

        if body is None:
            return

        # A body was selected, create the mouse joint
        self.mouse_joint = self.world.create_mouse_joint(
                body, 
                target = p,
                max_force = 1000.0*body.mass)
        body.awake = True

    def mouse_up(self, p):
        if self.mouse_joint:
            self.world.destroy_joint(self.mouse_joint)
            self.mouse_joint = None

    def mouse_move(self, p):
        if self.mouse_joint:
            self.mouse_joint.target = p

    # -- for the subclasses to implement --
    def key_down(self, key):
        """
        Callback indicating 'key' has been pressed down.
        The keys are mapped after pygame's style.

         from framework import Keys
         if key == Keys.K_z:
             ...
        """
        pass

    def key_up(self, key):
        """
        Callback indicating 'key' has been released.
        See key_down() for key information
        """
        pass

    def pre_step(self):
        """Called before a physics step."""
        pass

    def post_step(self):
        """Called after a physics step."""
        pass

    def fixture_destroyed(self, fixture):
        """A fixture is about to be destroyed"""
        pass

    def joint_destroyed(self, joint):
        """A joint is about to be destroyed"""
        pass

def main(test_class):
    """
    Loads the test class and executes it.
    """
    print("Loading %s..." % test_class.name)
    test = test_class()
    test.run()
