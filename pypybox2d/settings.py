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

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

# Global tuning constants based on meters-kilograms-seconds (MKS) units.

import sys
import math

PI = math.pi
try:
    EPSILON = sys.float_info.epsilon
    MAX_FLOAT = sys.float_info.max
except: # Python 2.5
    # NOTE: These could potentially be wrong, depending on your platform.
    EPSILON = 2.2204460492503131e-16
    MAX_FLOAT = 1.7976931348623157e+308

EPSILON_SQR = EPSILON**2

del math
del sys

# Global tuning constants based on meters-kilograms-seconds (MKS) units.
# Collision
# The maximum number of contact points between two convex shapes.
MAX_MANIFOLD_POINTS = 2

# This is used to fatten AABBs in the dynamic tree. This allows proxies
# to move by a small amount without triggering a tree adjustment.
# This is in meters.
AABB_EXTENSION = 0.1

# This is used to fatten AABBs in the dynamic tree. This is used to predict
# the future position based on the current displacement.
# This is a dimensionless multiplier.
AABB_MULTIPLIER = 2.0

# A small length used as a collision and constraint tolerance. Usually it is
# chosen to be numerically significant, but visually insignificant.
LINEAR_SLOP = 0.005

# A small angle used as a collision and constraint tolerance. Usually it is
# chosen to be numerically significant, but visually insignificant.
ANGULAR_SLOP = (2.0 / 180.0 * PI)

# The radius of the polygon/edge shape skin. This should not be modified. Making
# this smaller means polygons will have an insufficient buffer for continuous collision.
# Making it larger may create artifacts for vertex collision.
POLYGON_RADIUS = (2.0 * LINEAR_SLOP)

# Maximum number of sub-steps per contact in continuous physics simulation.
MAX_SUB_STEPS = 8


# Dynamics

# Maximum number of contacts to be handled to solve a TOI impact.
MAX_TOI_CONTACTS = 32

# TOI maximum number of iterations
TOI_MAX_ITERS = 20

# A velocity threshold for elastic collisions. Any collision with a relative linear
# velocity below this threshold will be treated as inelastic.
VELOCITY_THRESHOLD = 1.0

# The maximum linear position correction used when solving constraints. This helps to
# prevent overshoot.
MAX_LINEAR_CORRECTION = 0.2

# The maximum angular position correction used when solving constraints. This helps to
# prevent overshoot.
MAX_ANGULAR_CORRECTION = 8.0 / 180.0 * PI

# The maximum linear velocity of a body. This limit is very large and is used
# to prevent numerical problems. You shouldn't need to adjust this.
MAX_TRANSLATION = 2.0
MAX_TRANSLATION_SQR = MAX_TRANSLATION**2

# The maximum angular velocity of a body. This limit is very large and is used
# to prevent numerical problems. You shouldn't need to adjust this.
MAX_ROTATION = 0.5 * PI
MAX_ROTATION_SQR = MAX_ROTATION**2

# This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
# that overlap is removed in one time step. However using values close to 1 often lead
# to overshoot.
BAUMGARTE = 0.2
TOI_BAUMGARTE = 0.75

# Sleep

# The time that a body must be still before it will go to sleep.
TIME_TO_SLEEP = 0.5

# A body cannot sleep if its linear velocity is above this tolerance.
LINEAR_SLEEP_TOLERANCE = 0.01
LINEAR_SLEEP_TOLERANCE_SQR = LINEAR_SLEEP_TOLERANCE**2

# A body cannot sleep if its angular velocity is above this tolerance.
ANGULAR_SLEEP_TOLERANCE = (2.0 / 180.0 * PI)
ANGULAR_SLEEP_TOLERANCE_SQR = ANGULAR_SLEEP_TOLERANCE**2

# Contact
DEBUG_SOLVER = False

# Distance
DISTANCE_MAX_ITERS = 20
