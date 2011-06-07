#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simple benchmarking test modification of hello.py
"""

__version__ = "$Revision: 338 $"
__date__ = "$Date: 2011-05-20 09:21:24 -0400 (Fri, 20 May 2011) $"
# $Source$

USE_PSYCO=True

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

    world = b2.World()
    columns=5
    rows=16
    ground = world.create_static_body(
            shapes=[ 
                    b2.Edge((-40,0),(40,0)),
                    b2.Edge((20,0),(20,20)),
                ]
            ) 

    box=b2.Fixture(
            shape=b2.Polygon(box=(0.5,0.5)),
            density=1,
            friction=0.3)
    circle=b2.Fixture(
            shape=b2.Circle(0.5),
            density=1,
            friction=0.3)

    box_start=-10
    box_space=2.5
    circle_start=8
    circle_space=2.5
    for j in range(columns):
        for i in range(rows):
            world.create_dynamic_body(
                    fixtures=box,
                    position=(box_start+box_space*j, 0.752 + 1.54 * i)
                    )
            world.create_dynamic_body(
                    fixtures=circle,
                    position=(circle_start+circle_space*j, 0.752 + 1.54 * i)
                    )

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
