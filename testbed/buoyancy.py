#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
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

from __future__ import absolute_import

__version__ = "$Revision: 337 $"
__date__ = "$Date: 2011-05-19 16:44:08 -0400 (Thu, 19 May 2011) $"
# $Source$

from framework import (Framework, main)
from bridge import create_bridge
import pypybox2d as b2

class Buoyancy(Framework):
    """
    A test of the Buoyancy controller
    """
    name = "Buoyancy"
    description=""
    def __init__(self):
        Framework.__init__(self)
        
        world = self.world

        world.create_static_body(shapes=b2.Edge((-20, 0),( 20, 0))) 

        controller = world.create_buoyancy_controller(
                                        offset=15, normal=(0, 1), density=2, 
                                        linear_drag=2, angular_drag=1)
        
        for i, restitution in enumerate([0.0, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2]):
            body = self.world.create_dynamic_body(
                    position=(-10+3.0*i, 20), 
                    fixtures=b2.Fixture(shape=b2.Circle(radius=1.0), 
                                density=1.0, restitution=restitution)
                    )

            controller.add_body(body)

if __name__=="__main__":
    main(Buoyancy)

