#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simple benchmarking test modification of hello.py
"""

__version__ = "$Revision: 338 $"
__date__ = "$Date: 2011-05-20 09:21:24 -0400 (Fri, 20 May 2011) $"
# $Source$

import sys
import time
sys.path.extend(['..', '.'])

import pypybox2d as b2
from pypybox2d.common import *

def do_test(iters):
    start = time.time()
    world = b2.World((0, -10), True)

    ground = world.create_static_body(position=(0, -10))
    ground.create_polygon_fixture(box=(50, 10))

    body = world.create_dynamic_body(position=(0, 4))
    body.create_circle_fixture(1.0, density=1.0)
    #body.create_polygon_fixture(box=(1, 1), density=1.0, friction=0.3)

    init_time = start - time.time()

    range_ = range(iters)

    start = time.time()
    for i in range_:
        world.step(0.1, 10, 10)
        world.clear_forces()

    step_time = time.time() - start
    return init_time, step_time

print('-- %s --' % sys.version)

print('iters:\ttime\tper\tstep\tinit')
for i in (100, 1000, 10000, 100000, 1000000):
    init_time, step_time = do_test(i)
    elapsed = init_time + step_time
    print('%d:\t%.2f\t%.2g\t%.2f\t%.2f' % (i, elapsed, elapsed/i, step_time, init_time))
