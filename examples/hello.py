#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a simple example of building and running a simulation
using pypybox2d. Here we create a large ground box and a small dynamic 
circle.

NOTE:
There is no graphical output for this simple example, only text.
"""

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

import sys
sys.path.extend(['..', '.'])

import pypybox2d as b2
from pypybox2d.common import *

# Create the world, with gravity (0, -10), and allow bodies to sleep (they use 
# less CPU when not moving)
world = b2.World((0, -10), True)

# Create a static ground body
ground = world.create_static_body(position=(0, -10))

# And attach a Polygon to it. A Fixture holds the shape and contains
# such information as its density, friction, etc. The box(hx, hy) 
# parameter indicates for the polygon to be a box of half width
# 50 and half height 10. Since pypybox2d expects MKS units internally,
# the box is essentially 100m wide by 10m high. For a static body, this
# is just fine.
ground.create_polygon_fixture(box=(50, 10))

# Create a dynamic body in the world
body = world.create_dynamic_body(position=(0, 4))

# And attach a Circle to it of radius 1.0. Ensure that a density
# is set, otherwise the fixture would become static.
body.create_circle_fixture(1.0, density=1.0)

# # Or attach a polygon:
# body.create_polygon_fixture(box=(1, 1), density=1.0, friction=0.3)

# As a test, apply an impulse either linearly or angularly:
#body.apply_angular_impulse(100.0)
#body.apply_linear_impulse((10.0, 0), (0, 0))

# This is our little game loop.
for i in range(50):
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    world.step(0.1, 10, 10)

    # Clear applied body forces. We didn't apply any forces, but you
    # should know about this function. World also has an auto_clear_forces
    # option which will do this after every step for you.
    world.clear_forces()

    # Now print the position and angle of the body.
    print(body.position, body.angle)

