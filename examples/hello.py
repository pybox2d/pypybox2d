#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a simple example of building and running a simulation
using Box2D. Here we create a large ground box and a small dynamic box.

NOTE:
There is no graphical output for this simple example, only text.
"""

import sys
sys.path.append('..')

import pypybox2d as b2
from pypybox2d.common import *

world=b2.World((0, -10), True)
body=world.create_dynamic_body(position=(0, -1))
body.create_circle_fixture(1.0, density=1.0)

#body.apply_angular_impulse(10.0)
#body.apply_linear_impulse((10.0, 0), (0, 0))

# This is our little game loop.
for i in range(10):
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    world.step(0.1, 10, 10)

    # Clear applied body forces. We didn't apply any forces, but you
    # should know about this function.
    world.clear_forces()

    # Now print the position and angle of the body.
    print(body.position, body.angle)

