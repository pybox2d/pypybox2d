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

import math
from .settings import (EPSILON, MAX_FLOAT)
from sys import version_info

__all__ = (# Exceptions
           'PhysicsError', 'LockedError', 'EmptyFixtureError',

           # Constants
           'PI', 'EPSILON', 'MAX_FLOAT', 'NUMBER_TYPES',

           # Classes
           'Vec2', 'Mat22', 'AABB', 'Transform',
           'PyVec2', 'PyMat22', 'PyAABB', 'PyTransform',
           'Mat33', 'Sweep',

           # Functions
           'scalar_cross', 'min_vector', 'max_vector', 'clamp', 'clamp_vector',
           'next_power_of_two', 'is_power_of_two',
           'is_valid_float', 'inv_sqrt',
           'distance', 'distance_squared',

           # For python 2.5 compatibility:
           'property'
          )

if version_info >= (3,0,0):
    NUMBER_TYPES = (float, int)
else:
    NUMBER_TYPES = (float, long, int)

if version_info < (2,6,0):
    # Getter/setter syntax for < Python 2.5
    # Thanks to http://blog.devork.be/2008/04/xsetter-syntax-in-python-25.html
    import sys
    _property = property

    class property(_property):
        def __init__(self, fget, *args, **kwargs):
            self.__doc__ = fget.__doc__
            _property.__init__(self, fget, *args, **kwargs)

        def setter(self, fset):
            cls_ns = sys._getframe(1).f_locals
            for k, v in cls_ns.iteritems():
                if v == self:
                    propname = k
                    break
            cls_ns[propname] = property(self.fget, fset,
                                        self.fdel, self.__doc__)
            return cls_ns[propname]

else:
    property = property


del version_info
PI = math.pi
USE_PURE_PYTHON = False

class PhysicsError(Exception): pass
class LockedError(PhysicsError): pass
class EmptyFixtureError(PhysicsError): pass

class PyVec2(object):
    """A 2d column vector"""
    __slots__=['x', 'y']
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = float(x), float(y)

    __iter__ = lambda self: iter((self.x, self.y))
    def __repr__(self):
        return "Vec2(%g,%g)" % (self.x, self.y)
    def __abs__(self):
        return Vec2(abs(self.x), abs(self.y))
    def __len__(self):
        return 2
    def __neg__(self):
        return Vec2(-self.x, -self.y)
    def __copy__(self):
        return Vec2(self.x, self.y)
    copy = __copy__
    def __iadd__(self, other):
        ox, oy = other
        self.x += ox
        self.y += oy
        return self
    def __add__(self, other):
        return Vec2(self.x+other[0], self.y+other[1])
    def __sub__(self, other):
        return Vec2(self.x-other[0], self.y-other[1])
    def __rsub__(self, other):
        return Vec2(other[0]-self.x, other[1]-self.y)
    def __isub__(self, other):
        ox, oy = other
        self.x -= ox
        self.y -= oy
        return self
    def __imul__(self, other):
        if isinstance(other, NUMBER_TYPES):
            self.x *= other
            self.y *= other
        else:
            self.x *= other[0]
            self.y *= other[1]
        return self
    def __itruediv__(self, other):
        if isinstance(other, NUMBER_TYPES):
            self.x /= other
            self.y /= other
        else:
            self.x /= other[0]
            self.y /= other[1]
        return self
    def __mul__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec2(value*self.x, value*self.y)
        else:
            return self.dot(value)
    def __div__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec2(self.x/value, self.y/value)
        else:
            raise ValueError('Ambiguous operation')
    def __floordiv__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec2(self.x//value, self.y//value)
        else:
            raise ValueError('Ambiguous operation')
    def __nonzero__(self):
        return self.x!=0.0 or self.y!=0.0
    def __eq__(self, other):
        try:
            return (self.x==other[0] and self.y==other[1])
        except:
            return False
    def __ne__(self, other):
        try:
            return (self.x!=other[0] or self.y!=other[1])
        except:
            return True
    def __lt__(self, other):
        try:
            return (self.x < other[0] and self.y < other[1])
        except:
            return False
    def __gt__(self, other):
        try:
            return (self.x > other[0] and self.y > other[1])
        except:
            return False
    def __le__(self, other):
        try:
            return (self.x <= other[0] and self.y <= other[1])
        except:
            return False
    def __ge__(self, other):
        try:
            return (self.x >= other[0] and self.y >= other[1])
        except:
            return False
    def __getitem__(self, i):
        if i==0:
            return self.x
        elif i==1:
            return self.y
        else:
            raise IndexError('Index must be in (0,1)')
    def __setitem__(self, i, value):
        if i==0:
            self.x=float(value)
        elif i==1:
            self.y=float(value)
        else:
            raise IndexError('Index must be in (0,1)')
    def __rdiv__(self, left_value):
        return Vec2(left_value / self.x, left_value / self.y)
    def __getstate__(self):
        return [self.x, self.y]
    def __setstate__(self, value):
        self.x, self.y = value
    __idiv__=__itruediv__
    __rmul__=__mul__
    __truediv__=__div__
    __radd__=__add__
    __rtruediv__=__rdiv__

    @property
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)
    @property
    def length_squared(self):
        return self.x**2 + self.y**2
    @property
    def valid(self):
        return is_valid_float(self.x) and is_valid_float(self.y)

    def set(self, x, y):
        """
        Set the vector, copying the elements passed in.
        """
        self.x, self.y=float(x), float(y)
    def zero(self):
        """Zero the vector"""
        self.x, self.y=0.0, 0.0
    def _scalar_cross(self, value):
        """
        scalar x vector

        Perform the cross product on a scalar and this vector. In 2D this produces
        a vector.
        """
        return Vec2(-float(value) * self.y, float(value) * self.x)

    def cross(self, value):
        if isinstance(value, NUMBER_TYPES):
            # Perform the cross product on a vector and a scalar. In 2D this produces
            # a vector.
            return Vec2(float(value)*self.y, -float(value)*self.x)
        else:
            # Perform the cross product on two vectors. In 2D this produces a scalar.
            vx, vy = value
            return self.x * float(vy) - self.y * float(vx)
    def dot(self, value):
        vx, vy = value
        return self.x * float(vx) + self.y * float(vy)
    def normalize(self):
        length=self.length
        if length < EPSILON:
            return 0.0
        inv_length=1.0 / length
        self.x*=inv_length
        self.y*=inv_length
        return length
    def skew(self):
        return Vec2(-self.y, self.x)

class Vec3(object):
    """A 3d column vector"""
    __slots__=['x', 'y', 'z']
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x=float(x)
        self.y=float(y)
        self.z=float(z)

    __iter__ = lambda self: iter((self.x, self.y, self.z))
    def __repr__(self):
        return "Vec3(%g,%g,%g)" % (self.x, self.y, self.z)
    def __len__(self):
        return 3
    def __abs__(self):
        return Vec3(abs(self.x), abs(self.y), abs(self.z))
    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)
    def __copy__(self):
        return Vec3(self.x, self.y, self.z)
    copy = __copy__
    def __iadd__(self, other):
        vx, vy, vz = other
        self.x += vx
        self.y += vy
        self.z += vz
        return self
    def __add__(self, other):
        vx, vy, vz = other
        return Vec3(self.x+vx, self.y+vy, self.z+vz)
    def __sub__(self, other):
        vx, vy, vz = other
        return Vec3(self.x-vx, self.y-vy, self.z-vz)
    def __isub__(self, other):
        vx, vy, vz = other
        self.x -= vx
        self.y -= vy
        self.z -= vz
        return self
    def __rsub__(self, other):
        vx, vy, vz = other
        return Vec3(vx-self.x, vy-self.y, vz-self.z)
    def __imul__(self, other):
        if isinstance(other, NUMBER_TYPES):
            self.x *= other
            self.y *= other
            self.z *= other
        else:
            vx, vy, vz = other
            self.x *= vx
            self.y *= vy
            self.z *= vz
        return self
    def __itruediv__(self, other):
        if isinstance(other, NUMBER_TYPES):
            self.x /= other
            self.y /= other
            self.z /= other
        else:
            vx, vy, vz = other
            self.x /= vx
            self.y /= vy
            self.z /= vz
        return self
    def __mul__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec3(value*self.x, value*self.y, value*self.z)
        else:
            return self.dot(value)
    def __div__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec3(self.x/value, self.y/value, self.z/value)
        else:
            raise ValueError('Ambiguous operation')
    def __floordiv__(self, value):
        if isinstance(value, NUMBER_TYPES):
            return Vec3(self.x//value, self.y//value, self.z//value)
        else:
            raise ValueError('Ambiguous operation')
    def __nonzero__(self):
        return self.x!=0.0 or self.y!=0.0 or self.z!=0.0
    def __eq__(self, other):
        try:
            return (self.x==other[0] and self.y==other[1] and self.z==other[2])
        except:
            return False
    def __ne__(self, other):
        try:
            return (self.x!=other[0] or self.y!=other[1] or self.z!=other[2])
        except:
            return True
    def __lt__(self, other):
        try:
            return (self.x < other[0] and self.y < other[1] and self.z < other[2])
        except:
            return False
    def __gt__(self, other):
        try:
            return (self.x > other[0] and self.y > other[1] and self.z > other[2])
        except:
            return False
    def __le__(self, other):
        try:
            return (self.x <= other[0] and self.y <= other[1] and self.z <= other[2])
        except:
            return False
    def __ge__(self, other):
        try:
            return (self.x >= other[0] and self.y >= other[1] and self.z >= other[2])
        except:
            return False
    def __getitem__(self, i):
        if i==0:
            return self.x
        elif i==1:
            return self.y
        elif i==2:
            return self.z
        else:
            raise IndexError('Index must be in (0,1,2)')
    def __setitem__(self, i, value):
        if i==0:
            self.x = float(value)
        elif i==1: 
            self.y = float(value)
        elif i==2: 
            self.z = float(value)
        else:
            raise IndexError('Index must be in (0,1,2)')
    def __rdiv__(self, left_value):
        return Vec3(left_value / self.x, left_value / self.y, left_value / self.z)
    def __getstate__(self):
        return [self.x, self.y, self.z]
    def __setstate__(self, value):
        self.x, self.y, self.z=value
    __idiv__=__itruediv__
    __rmul__=__mul__
    __truediv__=__div__
    __rtruediv__=__rdiv__
    __radd__=__add__

    @property
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    @property
    def length_squared(self):
        return self.x**2 + self.y**2 + self.z**2
    @property
    def valid(self):
        return is_valid_float(self.x) and is_valid_float(self.y) and is_valid_float(self.z)

    def set(self, x, y, z):
        """
        Set the vector, copying the elements passed in.
        """
        self.x, self.y, self.z = x, y, z
    def zero(self):
        """Zero the vector"""
        self.x, self.y, self.z = 0.0, 0.0, 0.0
    def dot(self, value):
        vx, vy, vz = value
        return (self.x * float(vx)) + (self.y * float(vy)) + (self.z * float(vz))
    def normalize(self):
        length = self.length
        if length < EPSILON:
            return 0.0
        inv_length = 1.0 / length
        self.x *= inv_length
        self.y *= inv_length
        self.z *= inv_length
        return length
    def cross(self, other):
        vx, vy, vz = other
        sx, sy, sz = self
        return Vec3(sy * vz - sz * vy, 
                    sz * vx - sx * vz,
                    sx * vy - sy * vx)

class PyMat22(object):
    """
    2x2 matrix, stored in column-major order.
    """
    __slots__=['_col1', '_col2']
    def __init__(self, col1=(1, 0), col2=(0, 1)):
        self._col1, self._col2=Vec2(*col1), Vec2(*col2)

    def __str__(self):
        return \
"""
[ %.2g %.2g
%.2g %.2g ]
""" % (self._col1.x, self._col2.x,
   self._col1.y, self._col2.y )

    @property
    def col1(self):
        return Vec2(*self._col1)
    @col1.setter
    def col1(self, value):
        self._col1 = Vec2(*value)

    @property
    def col2(self):
        return Vec2(*self._col2)
    @col2.setter
    def col2(self, value):
        self._col2 = Vec2(*value)

    def __copy__(self):
        return Mat22(self._col1, self._col2)
    copy = __copy__

    def __mul__(self, vec):
        """
        Multiply a matrix times a vector. If a rotation matrix is provided,
        then this transforms the vector from one frame to another.
        """
        v0, v1 = vec
        return Vec2((self._col1.x * v0 + self._col2.x * v1),
                    (self._col1.y * v0 + self._col2.y * v1))
 
    def __abs__(self):
        return Mat22(abs(self._col1), abs(self._col2))
    def __add__(self, other):
        return Mat22(self._col1 + other[0], self._col2 + other[1])
    def __iadd__(self, other):
        self._col1 += other[0]
        self._col2 += other[1]
        return self
    def __sub__(self, other):
        return Mat22(self._col1 - other[0], self._col2 - other[1])
    def __isub__(self, other):
        self._col1 -= other[0]
        self._col2 -= other[1]
        return self
    def __eq__(self, other):
        try:
            return (self._col1==other[0] and self._col2==other[1])
        except:
            return False
    def __ne__(self, other):
        try:
            return (self._col1!=other[0] or self._col2!=other[1])
        except:
            return True
    def __lt__(self, other):
        try:
            return (self._col1 < other[0] and self._col2 < other[1])
        except:
            return False
    def __gt__(self, other):
        try:
            return (self._col1 > other[0] and self._col2 > other[1])
        except:
            return False
    def __le__(self, other):
        try:
            return (self._col1 <= other[0] and self._col2 <= other[1])
        except:
            return False
    def __ge__(self, other):
        try:
            return (self._col1 >= other[0] and self._col2 >= other[1])
        except:
            return False
    __iter__ = lambda self: iter((self._col1, self._col2))
    def __len__(self):
        return 2
    def __getitem__(self, i):
        if i==0:
            return Vec2(*self._col1)
        elif i==1:
            return Vec2(*self._col2)
        else:
            raise IndexError('Index must be in (0,1)')
    def __setitem__(self, i, value):
        if i==0:
            self._col1=Vec2(*value)
        elif i==1:
            self._col2=Vec2(*value)
        else:
            raise IndexError('Index must be in (0,1)')

    def set(self, other, col2=None):
        """
        Set the matrix in-place, copying the elements passed in.

        Can be used as .set(Transform()) or .set(col1, col2)
        """
        if col2 is not None:
            self._col1=Vec2(*other)
            self._col2=Vec2(*col2)
        else:
            self._col1=Vec2(*other.col1)
            self._col2=Vec2(*other.col2)

    def mul_t(self, other):
        """
        Transpose multiplication.

        If the argument is a vector, return:
          A^T * vector (where ^T is transpose)
        If the argument is a matrix, return:
          A^T * matrix
        """
        # TODO break this up
        if isinstance(other, (list, tuple, Vec2)):
            return Vec2(self._col1.dot(other), self._col2.dot(other))
        else:
            return Mat22(
                ((self._col1.dot(other.col1)), (self._col2.dot(other.col1))),
                ((self._col1.dot(other.col2)), (self._col2.dot(other.col2))),
                )
        
    @property
    def angle(self):
        return math.atan2(self._col1.y, self._col1.x)

    @angle.setter
    def angle(self, radians):
        """
        Make an orthonormal rotation matrix with 'angle' (in radians)
        """
        c=math.cos(radians)
        s=math.sin(radians)
        self._col1.x=c; self._col2.x=-s
        self._col1.y=s; self._col2.y=c

    @property
    def inverse(self):
        """
        Calculate the inverse matrix
        """ 
        # a b <=> col1.x col2.x
        # c d     col1.y col2.y
        a, b=self._col1.x, self._col2.x
        c, d=self._col1.y, self._col2.y
        det=a*d - b*c
        if det != 0.0:
            det = 1.0 / det

        return Mat22(( det * d, -det * c),
                     (-det * b,  det * a))

    def solve(self, vec):
        """ Solve A * x = vec, a column vector. This is more efficient
            than computing the inverse in one-shot cases."""
        a, b=self._col1.x, self._col2.x
        c, d=self._col1.y, self._col2.y
        det=a*d - b*c
        if det != 0.0:
            det = 1.0 / det

        return Vec2(det * (d * vec[0] - b * vec[1]),
                    det * (a * vec[1] - c * vec[0]))

    def set_identity(self):
        """
        [ 1 0
          0 1 ]
        """
        self._col1.x=1.0; self._col2.x=0.0
        self._col1.y=0.0; self._col2.y=1.0

    def set_zero(self):
        """
        [ 0 0
          0 0 ]
        """
        self._col1.x=0.0; self._col2.x=0.0
        self._col1.y=0.0; self._col2.y=0.0

class Mat33(object):
    """
    3x3 matrix, stored in column-major order.
    """
    __slots__=['col1', 'col2', 'col3']
    def __init__(self, col1=(1, 0, 0), col2=(0, 1, 0), col3=(0, 0, 1)):
        self.col1, self.col2, self.col3=Vec3(*col1), Vec3(*col2), Vec3(*col3)
    
    def __str__(self):
        return \
"""
[ %.2g %.2g %.2g
  %.2g %.2g %.2g
  %.2g %.2g %.2g ]
""" % (self.col1.x, self.col2.x, self.col3.x,
       self.col1.y, self.col2.y, self.col3.y,
       self.col1.z, self.col2.z, self.col3.z)

    def __abs__(self):
        return Mat33(abs(self.col1), abs(self.col2), abs(self.col3))
    def __mul__(self, vec):
        """
        Multiply a matrix times a vector. If a rotation matrix is provided,
        then this transforms the vector from one frame to another.
        """
        return Vec3((self.col1.x * vec[0] + self.col2.x * vec[1] + self.col3.x * vec[2]),
                    (self.col1.y * vec[0] + self.col2.y * vec[1] + self.col3.y * vec[2]),
                    (self.col1.z * vec[0] + self.col2.z * vec[1] + self.col3.z * vec[2]))

    __iter__ = lambda self: iter((self.col1, self.col2, self.col3))
    def __len__(self):
        return 3
    def __copy__(self):
        return Mat33(self.col1, self.col2, self.col3)
    copy = __copy__
    def __getitem__(self, i):
        if i==0:
            return self.col1
        elif i==1:
            return self.col2
        elif i==2:
            return self.col3
        else:
            raise IndexError('Index must be in (0,1,2)')
    def __setitem__(self, i, value):
        if i==0:
            self.col1=Vec3(value)
        elif i==1:
            self.col2=Vec3(value)
        elif i==2:
            self.col3=Vec3(value)
        else:
            raise IndexError('Index must be in (0,1,2)')

    def set(self, other, col2=None, col3=None):
        """
        Set the matrix, copying the elements passed in.

        Can be used as .set(Transform()) or .set(col1, col2)
        """
        if col2 is not None and col3 is not None:
            self.col1=Vec3(other)
            self.col2=Vec3(col2)
            self.col2=Vec3(col3)
        else:
            self.col1=Vec3(other.col1)
            self.col2=Vec3(other.col2)
            self.col3=Vec3(other.col3)

    def set_identity(self):
        """
        [ 1 0 0
          0 1 0
          0 0 1 ]
        """
        self.col1.x=1.0; self.col2.x=0.0; self.col3.x=0.0
        self.col1.y=0.0; self.col2.y=1.0; self.col3.y=0.0
        self.col1.z=0.0; self.col2.z=0.0; self.col3.z=1.0

    def set_zero(self):
        """
        [ 0 0 0 
          0 0 0
          0 0 0 ]
        """
        self.col1.x=0.0; self.col2.x=0.0; self.col3.x=0.0
        self.col1.y=0.0; self.col2.y=0.0; self.col3.y=0.0
        self.col1.z=0.0; self.col2.z=0.0; self.col3.z=0.0

    def solve3x3(self, b):
        """
        Solve A * x = b, where b is a column vector. This is more efficient
        than computing the inverse in one-shot cases.
        """
        det = self.col1.dot(self.col2.cross(self.col3))
        if det != 0.0:
            det = 1.0 / det
        return Vec3(
                det * b.dot(self.col2.cross(self.col3)),
                det * self.col1.dot(b.cross(self.col3)),
                det * self.col1.dot(self.col2.cross(b)),
                )

    def solve2x2(self, vec):
        """
        Solve A * x = Vec2, a 2d column vector. This is more efficient
        than computing the inverse in one-shot cases. Solve only the upper
        2-by-2 matrix equation.
        """
        a11, a12 = self.col1.x, self.col2.x
        a21, a22 = self.col1.y, self.col2.y
        det = a11 * a22 - a12 * a21
        if det != 0.0:
            det = 1.0 / det

        return Vec2(det * (a22 * vec[0] - a12 * vec[1]),
                    det * (a11 * vec[1] - a21 * vec[0]))

class PyTransform(object):
    """
    A Transform contains translation and rotation. It is used to represent
    the position and orientation of rigid frames.
    """
    __slots__=['_position', '_rotation']

    def __init__(self, position=(0, 0), rotation=((1, 0), (0, 1)), angle=None):
        self._position=Vec2(*position)
        if angle is not None:
            self._rotation=Mat22()
            self._rotation.angle=angle
        else:
            self._rotation=Mat22(*rotation)

    @property
    def position(self):
        """The offset of the transform"""
        return self._position
    
    @position.setter
    def position(self, position):
        self._position = Vec2(*position)

    @property
    def rotation(self):
        """The rotation matrix of the transform"""
        return Mat22(self._rotation.col1, self._rotation.col2)
    
    @rotation.setter
    def rotation(self, rotation):
        self._rotation = Vec2(*rotation)

    def __copy__(self):
        return Transform(self._position, self._rotation)
    copy = __copy__

    def __repr__(self):
        return 'Transform(position=%s, angle=%g)' % (
                self._position, self._rotation.angle)

    def set_identity(self):
        """Set this to the identity transform."""
        self._position=(0,0)
        self._rotation.set_identity()

    def mul_t(self, other):
        """
        Transpose multiplication.
        Takes either vector or another transform.
        Vector: A._rotation^T (vec - A.position)
        Transform: y2 = A.R' * (B.R * v1 + B.p - A.p) = (A.R' * B.R) * v1 + (B.p - A.p)
        """
        if isinstance(other, (list, tuple, Vec2)):
            return self._rotation.mul_t(other-self._position)
        elif isinstance(other, PyTransform):
            return PyTransform(other.position-self._position, self._rotation.mul_t(other._rotation))
        else:
            raise TypeError

    def __mul__(self, other):
        """
        
        """
        o0, o1 = other
        col1 = self._rotation.col1
        col2 = self._rotation.col2
        return Vec2(self._position.x + col1.x * o0 + col2.x * o1,
                    self._position.y + col1.y * o0 + col2.y * o1)

    def __eq__(self, other):
        try:
            return (self._position == other._position and self._rotation == other._rotation)
        except:
            return False
    def __ne__(self, other):
        try:
            return (self._position != other._position or self._rotation != other._rotation)
        except:
            return True

    @property
    def angle(self):
        return self._rotation.angle

    @angle.setter
    def angle(self, angle):
        self._rotation.angle=angle


class PyAABB(object):
    """An axis-aligned bounding box"""
    __slots__=['_upper_bound', '_lower_bound']
    def __init__(self, lower_bound=(0, 0), upper_bound=(0, 0)):
        self._lower_bound = Vec2(*lower_bound)
        self._upper_bound = Vec2(*upper_bound)

    def __repr__(self):
        return 'AABB(lower_bound=%s, upper_bound=%s)' \
                % (self._lower_bound, self._upper_bound)

    @property
    def lower_bound(self):
        return Vec2(*self._lower_bound)
    @lower_bound.setter
    def lower_bound(self, value):
        self._lower_bound.set(*value)

    @property
    def upper_bound(self):
        return Vec2(*self._upper_bound)
    @upper_bound.setter
    def upper_bound(self, value):
        self._upper_bound.set(*value)

    @property
    def valid(self):
        d=self._upper_bound-self._lower_bound
        area_valid=(d.x >= 0.0 and d.y >= 0.0)
        return area_valid and self._lower_bound.valid and self._upper_bound.valid

    __iter__ = lambda self: iter((self._lower_bound, self._upper_bound))
    def __len__(self):
        return 2
    def __getitem__(self, i):
        if i==0:
            return self._lower_bound
        elif i==1:
            return self._upper_bound
        else:
            raise IndexError('Index must be in (0,1)')
    def __setitem__(self, i, value):
        if i==0:
            self._lower_bound=Vec2(*value)
        elif i==1:
            self._upper_bound=Vec2(*value)
        else:
            raise IndexError('Index must be in (0,1)')
    def __eq__(self, other):
        try:
            return (self._lower_bound == other[0] and self._upper_bound == other[1])
        except:
            return False
    def __ne__(self, other):
        try:
            return (self._lower_bound != other[0] or self._upper_bound != other[1])
        except:
            return True

    def __copy__(self):
        return AABB(self._lower_bound, self._upper_bound)
    copy = __copy__

    def ray_cast(self, p1, p2, max_fraction):
        """
        Cast a ray against this AABB
        Returns: (hit, normal, fraction)
        @param p1 point 1
        @param p2 point 2
        @param max_fraction maximum fraction
        """
        # TODO raise exception on fail to make same as C AABB
        # From Real-time Collision Detection, p179.
        tmin = -MAX_FLOAT
        tmax = MAX_FLOAT

        p = p1
        d = Vec2(*p2) - p1
        abs_d = abs(d)
        normal=Vec2()

        for i in (0, 1):
            if abs_d[i] < EPSILON:
                # Parallel.
                if p[i] < self._lower_bound[i] or self._upper_bound[i] < p[i]:
                    return False, None, 0.0
            else:
                inv_d = 1.0 / d[i]
                t1 = (self._lower_bound[i] - p[i]) * inv_d
                t2 = (self._upper_bound[i] - p[i]) * inv_d

                # Sign of the normal vector.
                s = -1.0

                if t1 > t2:
                    t1, t2=t2, t1
                    s = 1.0

                # Push the min up
                if t1 > tmin:
                    normal.zero()
                    normal[i] = s
                    tmin = t1

                # Pull the max down
                tmax = min(tmax, t2)

                if tmin > tmax:
                    return False, None, 0.0

        # Does the ray start inside the box?
        # Does the ray intersect beyond the max fraction?
        if tmin < 0.0 or max_fraction < tmin:
            return False, None, 0.0

        # Intersection.
        return True, normal, tmin

    def test_overlap(self, b):
        """Test if this and another aabb overlap"""
        d1 = b._lower_bound - self._upper_bound
        d2 = self._lower_bound - b._upper_bound

        if d1.x > 0.0 or d1.y > 0.0:
            return False
        if d2.x > 0.0 or d2.y > 0.0:
            return False
        return True

    @property
    def center(self):
        """Get the center of the AABB."""
        return 0.5 * (self._lower_bound + self._upper_bound)

    @property
    def extents(self):
        """Get the extents of the AABB (half-widths)."""
        return 0.5 * (self._upper_bound - self._lower_bound)

    @property
    def perimeter(self):
        """Get the perimeter length"""
        wx = self._upper_bound.x - self._lower_bound.x
        wy = self._upper_bound.y - self._lower_bound.y
        return 2.0 * (wx + wy)

    def combine(self, aabb):
        """Combine an AABB into this one."""
        self._lower_bound = min_vector(self._lower_bound, aabb._lower_bound)
        self._upper_bound = max_vector(self._upper_bound, aabb._upper_bound)
        return self

    __iadd__=combine

    def combine_two(self, aabb1, aabb2):
        """Combine two AABBs into this one, ignoring the current AABB."""
        self._lower_bound = min_vector(aabb1._lower_bound, aabb2._lower_bound)
        self._upper_bound = max_vector(aabb1._upper_bound, aabb2._upper_bound)
        return self

    def __add__(self, other):
        ret=AABB(min_vector(self._lower_bound, other._lower_bound),
                 max_vector(self._upper_bound, other._upper_bound))
        return ret

    def contains(self, aabb):
        """Does this aabb contain the provided AABB?"""
        result = True
        result = result and (self._lower_bound.x <= aabb._lower_bound.x)
        result = result and (self._lower_bound.y <= aabb._lower_bound.y)
        result = result and (aabb._upper_bound.x <= self._upper_bound.x)
        result = result and (aabb._upper_bound.y <= self._upper_bound.y)
        return result

class Sweep(object):
    """
    This describes the motion of a body/shape for TOI computation.
    Shapes are defined with respect to the body origin, which may
    no coincide with the center of mass. However, to support dynamics
    we must interpolate the center of mass position.
    """
    __slots__=['local_center', 'c0', 'c', 'a0', 'a', 'alpha0']
    def __init__(self, local_center=(0, 0), c0=(0, 0), c=(0, 0), a0=0.0, a=0.0, alpha0=0.0):
        self.local_center = Vec2(*local_center) #< local center of mass position
        self.c0, self.c = Vec2(*c0), Vec2(*c)  #< center world positions
        self.a0, self.a = a0, a        #< world angles

        # Fraction of the current time step in the range [0,1]
        # c0 and a0 are the positions at alpha0.
        self.alpha0 = alpha0

    def __copy__(self):
        return Sweep(self.local_center, self.c0, self.c, self.a0, self.a, self.alpha0)
    copy = __copy__

    def __repr__(self):
        return 'Sweep(local_center=%s, c0=%s, c=%s, a0=%g, a=%g, alpha0=%g)' % (
                      self.local_center, self.c0, self.c, self.a0, self.a, self.alpha0)

    def get_transform(self, beta):
        """
        Get the interpolated transform at a specific time.
        beta: is a factor in [0,1], where 0 indicates alpha0.
        """

        xf = Transform(position=(1.0 - beta) * self.c0 + beta * self.c,
                       angle=(1.0 - beta) * self.a0 + beta * self.a)
        
        # Shift to origin
        xf.position -= xf._rotation * self.local_center
        return xf

    def advance(self, alpha):
        """
        Advance the sweep forward, yielding a new initial state.
        alpha: the new initial time.
        """
        assert(self.alpha0 < 1.0)
        beta = (alpha - self.alpha0) / (1.0 - self.alpha0)
        self.c0 = (1.0 - beta) * self.c0 + beta * self.c
        self.a0 = (1.0 - beta) * self.a0 + beta * self.a
        self.alpha0 = alpha

    def normalize(self):
        """Normalize the angles."""
        d=2.0 * PI * math.floor(self.a0 / (2.0 * PI))
        self.a0 -= d
        self.a -= d

def next_power_of_two(x):
    """"Next Largest Power of 2
    Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
    that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
    the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
    largest power of 2."""
    x=int(x)
    x = x | (x >> 1)
    x = x | (x >> 2)
    x = x | (x >> 4)
    x = x | (x >> 8)
    x = x | (x >> 16)
    return x+1

def is_power_of_two(x):
    return x > 0 and ((x & (x - 1)) == 0)

def is_valid_float(x):
    return not math.isnan(x)

def inv_sqrt(x):
    return 1.0 / math.sqrt(x)
def distance(p1, p2):
    c = p1 - p2
    return c.length
def distance_squared(p1, p2):
    c = p1 - p2
    return c.dot(c)

try:
    if USE_PURE_PYTHON:
        raise
    from ._common import Vec2
    from ._common import (scalar_cross, min_vector, max_vector, clamp, clamp_vector)
    from ._common import Mat22
    from ._common import Transform
    from ._common import AABB
    print('Using C extension')
except:
    print('Using Pure Python')
    Mat22 = PyMat22
    Transform = PyTransform
    AABB = PyAABB
    Vec2 = PyVec2
    def scalar_cross(scalar, vector):
        return vector._scalar_cross(scalar)
    def min_vector(v1, v2):
        return Vec2(min(v1[0], v2[0]), min(v1[1], v2[1]))
    def max_vector(v1, v2):
        return Vec2(max(v1[0], v2[0]), max(v1[1], v2[1]))
    def clamp_vector(value, low, high):
        return max_vector(low, min_vector(value, high))
    def clamp(value, low, high):
        return max(low, min(value, high))

if __name__=='__main__':
    a=Vec2()
    b=Vec2(2.5,3.1)
    assert(b == (2.5, 3.1))
    b=Vec2(*b)
    assert(b == (2.5, 3.1))
    c=Vec2(*a)
    print(a, b, c)
    assert(b.dot(b) == b*b)
    assert(b.dot(b) == (2.5**2 + 3.1**2))
    b+=Vec2(1,2)
    assert(b == (3.5, 5.1))
    b+=(1,1)
    assert(b == (4.5, 6.1))
    b*=2
    assert(b == (9, 12.2))
    a=Vec2(-1, -1)
    assert(abs(a) == (1, 1))
    assert((b*2)/2 == b)
    assert(0.5 * (b*2) == b)
    b/=2
    assert(b == (4.5, 6.1))

    a=Vec2(3, 3)
    a//=2
    assert(a == (1, 1))
    assert(a[0] == 1 and a[1] == 1)
    a[0]=2.0
    assert(a[0] == 2 and a[1] == 1)

    a=Vec2(1,1)
    b=Vec2(3,2)
    assert(a.cross(b) == (1*2 - 1*3))
    print(scalar_cross(1, b))
    assert(scalar_cross(1, b) == (-1.0 * 2.0, 1.0 * 3.0))
    assert(scalar_cross(1.0, b) == (-1.0 * 2.0, 1.0 * 3.0))

    a=Vec3()
    b=Vec3(2.5,3.1,1.0)
    assert(b == (2.5, 3.1, 1.0))
    b=Vec3(*b)
    assert(b == (2.5, 3.1, 1.0))
    c=Vec3(*a)
    assert(a+b == b)
    assert(a+c == c)
    assert(b+c == c+b)
    assert(b.dot(b) == b*b)
    assert(b.dot(b) == (2.5**2 + 3.1**2 + 1.0))
    b+=Vec3(1,2,3)
    assert(b == (3.5,5.1,4))
    b+=(1,1,3)
    assert(b == (4.5,6.1,7))
    b*=2
    assert((b*2)/2 == b)
    b/=2
    assert(b == (4.5,6.1,7))

    a=Mat22()
    a.angle=45 * PI / 180.0
    assert(a.angle*180.0/PI == 45)

    a=Mat22()
    print(a)
    assert(a.solve([1,1])==(1,1))
    assert(a.inverse == a)
    a=Mat33()
    print(a)
    assert(a.solve2x2([1,1])==(1,1))
    assert(is_power_of_two(256))
    assert(not is_power_of_two(55))
    assert(next_power_of_two(2) == 4)
    assert(next_power_of_two(8) == 16)

    a=AABB((-1, -1), (1, 1))
    print(a)
    assert(a.ray_cast((-2,0), (-1, 0), 1.0) == (True, (-1,0), 1.0))
    assert(a.center == (0, 0))
    assert(a.perimeter == 2*4)
    a.extents
    b=AABB((-0.5, -0.5), (0.5, 0.5))
    assert(a.contains(b))
    assert(not b.contains(a))
    
    c=a + b
    assert(c == a)
    assert(c != b)

    assert((1,1) + Vec2(1,1) == (2, 2))
    assert((1,1) - Vec2(1,1) == (0, 0))
    assert(2.0 / Vec2(1,1) == (2, 2))
    assert(2.0 * Vec2(1,1) == (2, 2))
    assert(Vec2(1,1) + (1,1) == (2,2))
    assert(Vec2(1,1) - (1,1) == (0,0))
    assert(Vec2(1,1) / 2.0 == (0.5,0.5))
    assert(Vec2(1,1) // 2.0 == (0,0))
    assert(Vec2(1,1) * 2.0 == (2,2))
    assert(-Vec2(1,1) == (-1,-1))

    assert((1,1,1) + Vec3(1,1,1) == (2,2,2))
    assert((1,1,1) - Vec3(1,1,1) == (0,0,0))
    assert(2.0 / Vec3(1,1,1) == (2,2,2))
    assert(2.0 * Vec3(1,1,1) == (2,2,2))
    assert(Vec3(1,1,1) + (1,1,1) == (2,2,2))
    assert(Vec3(1,1,1) - (1,1,1) == (0,0,0))
    assert(Vec3(1,1,1) / 2.0 == (0.5,0.5,0.5))
    assert(Vec3(1,1,1) // 2.0 == (0,0,0))
    assert(Vec3(1,1,1) * 2.0 == (2,2,2))
    assert(-Vec3(1,1,1) == (-1,-1,-1))
    a=Vec2(1, 1)
    b=Vec2(0, 0)
    assert(Vec2(1, 1) == Vec2(1, 1))
    assert(Vec2(1, 1) != Vec2(0, 0))
    assert(Vec3(1, 1, 1) == Vec3(1, 1, 1))
    assert(Vec3(1, 1, 1) != Vec3(0, 0, 0))
    assert(AABB(a, b) != AABB(b, a))
    assert(AABB(a, b) == AABB(a, b))

    assert(Mat33((1, 0, 0), (0, 1, 0), (0, 0, 1)) * Vec3(0, 0, 0) == (0, 0, 0))
    assert(Mat33((1, 0, 0), (0, 1, 0), (0, 0, 1)) * Vec3(1, 2, 3) == (1, 2, 3))
    assert(Mat22((1, 0), (0, 1)) * Vec2(0, 0) == (0, 0))
    assert(Mat22((1, 0), (0, 1)) * Vec2(1, 1) == (1, 1))
