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

from framework import *

class Empty(Framework):
    """You can use this class as an outline for your tests.

    """
    name = "Empty" # Name of the class to display
    description="The description text goes here"
    def __init__(self):
        """ 
        Initialize all of your objects here.
        Be sure to call the Framework's initializer first.
        """
        super(Empty, self).__init__()

        # Initialize all of the objects

    def key_down(self, key):
        """
        The key is from Keys.K_*
        (e.g., if key == Keys.K_z: ... )
        """
        pass

    def key_up(self, key):
        pass

    def pre_step(self):
        """Called before every physics step."""
        # do stuff
        pass

    def post_step(self):
        """Called after every physics step."""
        # do stuff
        self.print_("*** Base your own testbeds on me! ***")

    def fixture_destroyed(self, shape):
        """
        Callback indicating 'fixture' is to be destroyed
        """
        pass

    def joint_destroyed(self, joint):
        """
        The joint passed in was removed.
        """
        pass

    # More functions can be changed to allow for contact monitoring and such.
    # See the other testbed examples for more information.

if __name__=="__main__":
    main(Empty)

