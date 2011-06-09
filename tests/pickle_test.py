import sys
sys.path.append('..')

import unittest
import pypybox2d
from pypybox2d import *
import pickle

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

        for x in range(5):
            for y in range(2):
                pickled = pickle.dumps(Vec2(x, y))
                assert(pickle.loads(pickled) == (x, y))

    def _test_mat22(self, Mat22):
        pypybox2d.common.Mat22 = Mat22
        for x in range(5):
            for y in range(2):
                col1 = (x, y)
                col2 = (y, x)
                pickled = pickle.dumps(Mat22(col1, col2))
                assert(pickle.loads(pickled) == (col1, col2))

    def _test_transform(self, Transform):
        pypybox2d.common.Transform = Transform
        for x in range(5):
            for y in range(2):
                pos = (x, y)
                rot = Mat22(angle=(x * 3.14 / 180))
                pickled = pickle.dumps(Transform(pos, rot))
                assert(pickle.loads(pickled) == Transform(pos, rot))

    def _test_aabb(self, AABB):
        pypybox2d.common.AABB = AABB
        for x in range(5):
            for y in range(2):
                col1 = (x, y)
                col2 = (y, x)
                pickled = pickle.dumps(AABB(col1, col2))
                assert(pickle.loads(pickled) == AABB(col1, col2))

    def test_Vec2(self):
        assert(Vec2(0, 0) == PyVec2(0, 0))
        self._test_vec2(Vec2)
        if Vec2 != PyVec2:
            self._test_vec2(PyVec2)

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

if __name__ ==  '__main__':
    unittest.main()
