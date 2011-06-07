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

import os
try:
    from distribute_setup import use_setuptools
    use_setuptools()

    import setuptools
    from setuptools import (setup, Extension)
    setuptools_version=setuptools.__version__
    print('Using setuptools (version %s).' % setuptools_version)
except:
    from distutils.core import (setup, Extension)
    print('Using distutils.')

__author__='Ken Lauer'
__license__='zlib'
__date__="$Date: 2011-03-31 17:11:51 -0400 (Thu, 31 Mar 2011) $"
__version__="$Revision: 331 $"

package_name = 'pypybox2d'
major_version = '2.1'
major_release = False

if not major_release:
    try:
        revision = '-r%s' % __version__.split(' ')[1]
    except:
        revision = ''

    version = '%sdev%s' % (major_version, revision)
else:
    version = major_version

src_path = '%s/src' % package_name
c_source_files = ["vec2module.c", 
                  "mat22module.c", 
                  "aabbmodule.c", 
                  "transform.c",
                  "commonmodule.c",]
c_source_files = [os.path.join(src_path, fn) for fn in c_source_files]

pybox2d_url = "http://pybox2d.googlecode.com/"
box2d_url = "http://www.box2d.org"
pygame_url = "http://www.pygame.org"

LONG_DESCRIPTION = \
"""%s

   An [optionally] pure Python 2D physics library.

   After installing please be sure to try out the testbed examples.
   The demos require pygame: %s

   pybox2d homepage: %s
   Box2D homepage: %s
    """ % (package_name, pygame_url, pybox2d_url, box2d_url)

CLASSIFIERS = [
    "Development Status :: 3 - Alpha",
    "Topic :: Software Development :: Libraries",
    "Topic :: Software Development :: Libraries :: Python Modules",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: zlib/libpng License",
    "Operating System :: OS Independent",
    "Programming Language :: Python :: 2.5",
    "Programming Language :: Python :: 2.6",
    "Programming Language :: Python :: 2.7",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.0",
    "Programming Language :: Python :: 3.1",
    "Programming Language :: Python :: 3.2",
    ]

setup(name=package_name,
      version=version,
      author=__author__,
      description='2D Physics in Pure Python',
      license='zlib',
      long_description=LONG_DESCRIPTION,
      classifiers=CLASSIFIERS,
      url=pybox2d_url,
      test_suite='tests',
      platforms='any',
      packages=[package_name],
      package_dir={ package_name : package_name },
      ext_modules=[Extension("%s._common" % package_name, c_source_files)],
     )
