#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
# Python port by Ken Lauer / http://pybox2d.googlecode.com
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
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from . import world
from . import fixture
from . import body
from . import shapes
from . import joints
from . import dynamictree

from .world import World
from .fixture import Fixture
from .body import Body
from .shapes import (Circle, Polygon, Edge, Loop)
from .joints import (DistanceJoint, RevoluteJoint, FrictionJoint, PrismaticJoint,
                    WeldJoint, RopeJoint, WheelJoint, MouseJoint, PulleyJoint, GearJoint)
from .controllers import (Controller, BuoyancyController)
from .common import (
            # Exceptions
           PhysicsError, LockedError, EmptyFixtureError,

           # Constants
           PI, EPSILON, MAX_FLOAT, NUMBER_TYPES,

           # Classes
           Vec2, Mat22, AABB, Transform,
           PyVec2, PyMat22, PyAABB, PyTransform,
           Mat33, Sweep,

           # Functions
           scalar_cross, min_vector, max_vector, clamp, clamp_vector,
           next_power_of_two, is_power_of_two,
           is_valid_float, inv_sqrt,
           distance, distance_squared
          )
# Note: the ugliness above is because Python 2.5 does not support
#  from .common import *
