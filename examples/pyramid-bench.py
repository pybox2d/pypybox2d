#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simple benchmarking test modification of hello.py
"""

__version__ = "$Revision: 338 $"
__date__ = "$Date: 2011-05-20 09:21:24 -0400 (Fri, 20 May 2011) $"
# $Source$

USE_PSYCO=False

if USE_PSYCO:
    try:
        import psyco
        psyco.full()
    except Exception as ex:
        print('Failed to import psyco: %s' % ex)
    else:
        print('Using psyco')
import sys
import time
from copy import copy
sys.path.extend(['..', '.'])

import pypybox2d as b2
from pypybox2d.common import *

def do_test(iters):
    start = time.time()
    world = b2.World((0, -10), True)


    world.create_static_body(shapes=b2.Edge((-40, 0), (40, 0)))

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
            world.create_dynamic_body(position=y, fixtures=fixture)
            y += delta_y

        x += delta_x
    init_time = time.time() - start

    range_ = range(iters)

    TIMESTEP = 1.0 / 60
    start = time.time()
    for i in range_:
        world.step(TIMESTEP, 10, 10)
        world.clear_forces()

    step_time = time.time() - start
    return init_time, step_time

print('-- %s --' % sys.version)

print('iters:\ttime\tper\tstep\tinit')
for i in (10, 50, 100, 1000, 10000, 100000, 1000000):
    init_time, step_time = do_test(i)
    elapsed = init_time + step_time
    print('%d:\t%.2f\t%.2g\t%.2f\t%.2f' % (i, elapsed, elapsed/i, step_time, init_time))
