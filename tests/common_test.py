import sys
sys.path.append('..')

import unittest
import pypybox2d
from pypybox2d import *
from copy import copy

#import pypybox2d.objgraph as objgraph
#objgraph.show_most_common_types()

#print(Vec2(x=1.0, y=2.0))
#print(Vec2())

#print(Vec2(x=1.0/3, y=2.0))

def almost_equal(v1, v2):
    if isinstance(v1, Vec2):
        return abs(v1[0] - v2[0]) < EPSILON and abs(v1[1] - v2[1]) < EPSILON
    else:
        return (v1 - v2) < EPSILON

class common_test(unittest.TestCase):
    def setUp(self):
        pass

    def _test_vec2(self, Vec2):
        pypybox2d.common.Vec2 = Vec2
        v = Vec2(3.0, 4.0)
        v = v + (1, 2)
        v = v - (1, 1)
        v = v / 2
        v.length
        v.length_squared
     
        v = Vec2(3.0, 4.0)
        #print(v)
        #print(v.x, v.y)
        #print(v.length)
        #print(v.length_squared)
        #print(v)
        assert((v + v).x == 6.0)
        assert((v + v).y == 8.0)
        #print('adding tuple')
        assert((v + (3, 4)).x == 6.0)
        assert((v + (3, 4)).y == 8.0)
        #print((v - (3, 4)))
        assert((v - (3, 4)) == Vec2(0, 0))
        #assert((v * 3) != v.length_squared)
        assert((v * (3, 4)) == v.length_squared)
        assert((v * v) == v.length_squared)
        #assert((v + (3, 'x')).y == 8.0)
        #assert((v + (3, 4, 5)).y == 8.0)
        #
        assert(((3, 4) * v) == v.length_squared)
        assert(v.length_squared == v * v)
        assert(v.length_squared == v.dot(v))


        assert((1,1) + Vec2(1,1) == Vec2(2, 2))
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


        a=Vec2()
        b=Vec2(2.5,3.1)
        assert(2.5 in b)
        assert(3.1 in b)
        assert(1.0 not in b)
        assert(b == (2.5, 3.1))
        b=Vec2(*b)
        assert(b == (2.5, 3.1))
        c=Vec2(*a)
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
        #print(b, (b*2), 0.5 * (b*2))
        assert(0.5 * (b*2) == b)
        b/=2
        assert(b == (4.5, 6.1))

        a=Vec2(3, 3)
        assert(a == copy(a))
        assert(copy(a) == a == a.copy())
        a//=2
        #print(a)
        assert(a == (1, 1))
        assert(a[0] == 1 and a[1] == 1)
        a[0]=2.0
        assert(a[0] == 2 and a[1] == 1)

        a=Vec2(1,1)
        b=Vec2(3,2)
        assert(a.cross(b) == (1*2 - 1*3))
        #print(scalar_cross(1, b))
        assert(scalar_cross(1, b) == (-1.0 * 2.0, 1.0 * 3.0))
        assert(scalar_cross(1.0, b) == (-1.0 * 2.0, 1.0 * 3.0))

        i = 5
        a.set(i, i)
        a.zero()
        #print(a)
        #print(a.normalize())
        a.normalize()
        a.set(3, 4)
        #print(a.normalize())
        a.normalize()
        assert(copy(a) == a == a.copy())

    def _test_mat22(self, Mat22):
        pypybox2d.common.Mat22 = Mat22
        a=Mat22()
        a.angle=45 * PI / 180.0
        assert(a.angle*180.0/PI == 45)

        a=Mat22()
        assert(a.solve([1,1])==(1,1))
        assert(a.inverse == a)
        assert(a == a)
        assert(a != ((5, 0), (0, 1)))
        b = Mat22((0,0), (0,0))
        assert(b <= a)
        assert(b[0] == (0,0))
        assert(a[0] == (1,0))
        assert(a[1] == (0,1))
        assert((1, 0) in a)
        assert((0, 1) in a)
        assert((0, 0) not in a)
        a[0] = (1, 2)
        assert(a[0] == (1, 2))
        a[1] = (2, 3)
        assert(a[0] == (1, 2))
        assert(a[1] == (2, 3))
        assert(len(a) == 2)

        a[0] = (-1, -2)
        a[1] = (-2, -3)
        a = abs(a)
        assert(a[0] == (1, 2))
        assert(a[1] == (2, 3))

        a[0] += (-1, -2)
        a[1] += (-2, -3)
        assert(a[0] == (0, 0))
        assert(a[1] == (0, 0))

        a += ((1, 2), (3, 4))
        #print(a[0])
        assert(a[0] == (1, 2))
        assert(a[1] == (3, 4))
        a -= ((1, 2), (3, 4))
        assert(a == ((0, 0), (0, 0)))
        assert(a != ((1, 0), (0, 1)))

        a = Mat22()
        if not isinstance(a, PyMat22):
            try:
                a._col1 = None
                a._col2 = None
            except AttributeError:
                # good, read-only (py3k)
                pass
            except TypeError:
                # good, read-only (2.0)
                pass
            else:
                assert(False) # col1/col2 should be RO

        a._col1.x = 0.0
        a._col2.y = 0.0
        a._col1.x = 0.0
        a._col2.y = 0.0
        assert(a == ((0,0), (0,0)))

        a.set((1, 2), (3, 4))
        assert(a == ((1, 2), (3, 4)))
        
        assert(copy(a) == a == a.copy())
        a.set(a)
        assert(a == ((1, 2), (3, 4)))

        a.col1 += (0, 0)
        a.col2 += (0, 0)

    def _test_transform(self, Transform):
        pypybox2d.common.Transform = Transform
        xf = Transform(angle=PI / 2.0)
        #print(xf)
        v = Vec2(1.0, 0.0)
        #print(xf * v)
        assert(almost_equal(xf * v, (0, 1)))
        xfp = PyTransform(angle=PI / 2.0)
        #print(xfp.mul_t((0, 1)), xf.mul_t((0, 1)))
        assert(xfp.mul_t((0, 1)) == xf.mul_t((0, 1)))
        assert(xfp.mul_t(xfp)._rotation == xf.mul_t(xf)._rotation)
        assert(xfp.mul_t(xfp)._position == xf.mul_t(xf)._position)
        xfp4 = PyTransform(angle=PI / 4.0)
        xf4 = Transform(angle=PI / 4.0)
        assert(xfp.mul_t(xfp4)._rotation == xf.mul_t(xf4)._rotation)
        assert(xfp.mul_t(xfp4)._position == xf.mul_t(xf4)._position)
        assert(xfp4.mul_t(xfp)._rotation == xf4.mul_t(xf)._rotation)
        assert(xfp4.mul_t(xfp)._position == xf4.mul_t(xf)._position)
        assert(xf4 == copy(xf4))  # transform

    def _test_aabb(self, AABB):
        pypybox2d.common.AABB = AABB
        a=AABB((-1, -1), (1, 1))
        #print(a)
        #assert(a.ray_cast((-2,0), (-1, 0), 1.0) == (True, (-1,0), 1.0))
        assert(a.center == (0, 0))
        assert(a.perimeter == 2*4)
        a.extents
        b=AABB((-0.5, -0.5), (0.5, 0.5))
        assert(a.contains(b))
        assert(not b.contains(a))
        
        c=a + b
        assert(c == a)
        assert(c != b)
        assert(c == copy(c))  # transform
        assert(c != copy(b))  # transform
        
        b.lower_bound += (0, 0)

    def test_Vec2(self):
        assert(Vec2(0, 0) == PyVec2(0, 0))
        self._test_vec2(Vec2)
        if Vec2 != PyVec2:
            self._test_vec2(PyVec2)

    def test_functions(self):
        assert(min_vector((1, 1), (0, 0)) == (0, 0))
        assert(max_vector((1, 1), (0, 0)) == (1, 1))

        assert(clamp(0.1, -10.1, 10.1) == 0.1)
        assert(clamp(20.1, -10.1, 10.1) == 10.1)
        assert(clamp(-20.1, -10.1, 10.1) == -10.1)

    def test_Mat22(self):
        self._test_mat22(Mat22)
        if Mat22 != PyMat22:
            self._test_mat22(PyMat22)

    def test_Transform(self):
        self._test_transform(Transform)
        if Transform != PyTransform:
            self._test_transform(PyTransform)

    def test_AABB(self):
        self._test_aabb(AABB)
        if AABB != PyAABB:
            self._test_aabb(PyAABB)

#objgraph.show_most_common_types()

if __name__ ==  '__main__':
    unittest.main()
