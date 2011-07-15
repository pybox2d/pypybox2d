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

__all__ = ('toi_calls', 'toi_max_root_iters', 'toi_root_iters', 'toi_max_iters', 'toi_iters', 
           'UNKNOWN', 'FAILED', 'OVERLAPPED', 'TOUCHING', 'SEPARATED',
           'SeparationFunction', 
           'time_of_impact')

__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from copy import copy
from .settings import (LINEAR_SLOP, TOI_MAX_ITERS) #, MAX_POLYGON_VERTICES
from . import distance
from .shapes import Shape

toi_calls = 0
toi_max_root_iters = 0
toi_root_iters = 0
toi_max_iters = 0
toi_iters = 0

UNKNOWN, FAILED, OVERLAPPED, TOUCHING, SEPARATED=range(5)

class SeparationFunction(object):
    POINTS, FACE_A, FACE_B = range(3)
    def __init__(self, proxy_a, sweep_a, proxy_b, sweep_b):
        self.proxy_a = proxy_a
        self.proxy_b = proxy_b
        self.sweep_a = sweep_a
        self.sweep_b = sweep_b

    def initialize(self, cache, t1):
        count = cache.count
        assert(0 < count < 3)
        proxy_a = self.proxy_a
        proxy_b = self.proxy_b

        xf_a = self.sweep_a.get_transform(t1)
        xf_b = self.sweep_b.get_transform(t1)

        index_a0, index_b0 = cache.indices[0]
        if count == 1:
            self.type = SeparationFunction.POINTS
            self.find_min_separation = self.find_min_separation_points
            self.evaluate = self.evaluate_points

            local_point_a = self.proxy_a._vertices[index_a0]
            local_point_b = self.proxy_b._vertices[index_b0]
            point_a = xf_a * local_point_a
            point_b = xf_b * local_point_b
            self.axis = point_b - point_a
            s = self.axis.normalized
            return s

        index_a1, index_b1 = cache.indices[1]
        if index_a0 == index_a1:
            # Two points on B and one on A.
            self.type = SeparationFunction.FACE_B
            self.find_min_separation = self.find_min_separation_face_b
            self.evaluate = self.evaluate_face_b

            local_point_b1 = proxy_b._vertices[index_b0]
            local_point_b2 = proxy_b._vertices[index_b1]

            self.axis = (local_point_b2 - local_point_b1).cross(1.0).normalized
            normal = xf_b._rotation * self.axis

            self.local_point = 0.5 * (local_point_b1 + local_point_b2)
            point_b = xf_b * self.local_point

            local_point_a = proxy_a._vertices[index_a0]
            point_a = xf_a * local_point_a

            s = (point_a - point_b).dot(normal)
            if s < 0.0:
                self.axis = -self.axis
                s = -s
            return s
        else:
            # Two points on A and one or two points on B.
            self.type = SeparationFunction.FACE_A
            self.find_min_separation = self.find_min_separation_face_a
            self.evaluate = self.evaluate_face_a

            local_point_a1 = self.proxy_a._vertices[index_a0]
            local_point_a2 = self.proxy_a._vertices[index_a1]
            
            self.axis = (local_point_a2 - local_point_a1).cross(1.0).normalized
            normal = xf_a._rotation * self.axis

            self.local_point = 0.5 * (local_point_a1 + local_point_a2)
            point_a = xf_a * self.local_point

            local_point_b = self.proxy_b._vertices[index_b0]
            point_b = xf_b * local_point_b

            s = (point_b - point_a).dot(normal)
            if s < 0.0:
                self.axis = -self.axis
                s = -s
            return s

    def find_min_separation_points(self, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)

        axis_a = xf_a._rotation.mul_t( self.axis)
        axis_b = xf_b._rotation.mul_t(-self.axis)

        index_a = self.proxy_a.get_support(axis_a)
        index_b = self.proxy_b.get_support(axis_b)

        local_point_a = self.proxy_a._vertices[index_a]
        local_point_b = self.proxy_b._vertices[index_b]
        
        point_a = xf_a * local_point_a
        point_b = xf_b * local_point_b

        separation = (point_b - point_a).dot(self.axis)
        return separation, index_a, index_b

    def find_min_separation_face_a(self, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)
        normal = xf_a._rotation * self.axis
        point_a = xf_a * self.local_point

        axis_b = xf_b._rotation.mul_t(-normal)
        
        index_a = -1
        index_b = self.proxy_b.get_support(axis_b)

        local_point_b = self.proxy_b.vertices[index_b]
        point_b = xf_b * local_point_b

        separation = (point_b - point_a).dot(normal)
        return separation, index_a, index_b

    def find_min_separation_face_b(self, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)
        normal = xf_b._rotation * self.axis
        point_b = xf_b * self.local_point

        axis_a = xf_a._rotation.mul_t(-normal)

        index_b = -1
        index_a = self.proxy_a.get_support(axis_a)

        local_point_a = self.proxy_a._vertices[index_a]
        point_a = xf_a * local_point_a

        separation = (point_a - point_b).dot(normal)
        return separation, index_a, index_b

    def evaluate_points(self, index_a, index_b, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)

        local_point_a = self.proxy_a._vertices[index_a]
        local_point_b = self.proxy_b._vertices[index_b]

        point_a = xf_a * local_point_a
        point_b = xf_b * local_point_b
        separation = (point_b - point_a).dot(self.axis)

        return separation

    def evaluate_face_a(self, index_a, index_b, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)

        normal = xf_a._rotation * self.axis
        point_a = xf_a * self.local_point

        local_point_b = self.proxy_b._vertices[index_b]
        point_b = xf_b * local_point_b

        separation = (point_b - point_a).dot(normal)
        return separation

    def evaluate_face_b(self, index_a, index_b, t):
        xf_a = self.sweep_a.get_transform(t)
        xf_b = self.sweep_b.get_transform(t)

        normal = xf_b._rotation * self.axis
        point_b = xf_b * self.local_point

        local_point_a = self.proxy_a._vertices[index_a]
        point_a = xf_a * local_point_a

        separation = (point_a - point_b).dot(normal)
        return separation

def time_of_impact(proxy_a, proxy_b, sweep_a, sweep_b, t_max):
    """
    Compute the upper bound on time before two shapes penetrate. Time is represented as
    a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
    non-tunneling collision. If you change the time interval, you should call this function
    again.

    Note: use distance.shape_distance to compute the contact point and normal at the time of impact.
    """
    global toi_calls
    global toi_max_root_iters
    global toi_root_iters
    global toi_max_iters
    global toi_iters
    # CCD via the local separating axis method. This seeks progression
    # by computing the largest time at which separation is maintained.

    if not isinstance(proxy_a, distance.DistanceProxy):
        if not isinstance(proxy_a, Shape):
            raise ValueError('proxy_a: Must be either a DistanceProxy or a Shape')
        proxy_a = distance.DistanceProxy(proxy_a, 0)
    if not isinstance(proxy_b, distance.DistanceProxy):
        if not isinstance(proxy_b, Shape):
            raise ValueError('proxy_b: Must be either a DistanceProxy or a Shape')
        proxy_b = distance.DistanceProxy(proxy_b, 0)

    toi_calls += 1

    state = UNKNOWN
    output_t = t_max

    # Large rotations can make the root finder fail, so we normalize the
    # sweep angles.
    sweep_a = copy(sweep_a)
    sweep_b = copy(sweep_b)

    sweep_a.normalize()
    sweep_b.normalize()

    total_radius = proxy_a._radius + proxy_b._radius
    target = max(LINEAR_SLOP, total_radius - 3.0 * LINEAR_SLOP)
    tolerance = 0.25 * LINEAR_SLOP
    assert(target > tolerance)

    t1 = 0.0
    iter_ = 0

    # Prepare input for distance query.
    cache = distance.SimplexCache()

    # The outer loop progressively attempts to compute new separating axes.
    # This loop terminates when an axis is repeated (no progress is made).
    while True:
        xf_a = sweep_a.get_transform(t1)
        xf_b = sweep_b.get_transform(t1)

        # Get the distance between shapes. We can also use the results
        # to get a separating axis.
        dist_info = distance.shape_distance(proxy_a, proxy_b, xf_a, xf_b, False, cache)
        _point_a, _point_b, out_distance, _iterations = dist_info

        # If the shapes are overlapped, we give up on continuous collision.
        if out_distance <= 0.0:
            # Failure!
            state = OVERLAPPED
            output_t = 0.0
            break

        if out_distance < target + tolerance:
            # Victory!
            state = TOUCHING
            output_t = t1
            break

        # Initialize the separating axis.
        fcn = SeparationFunction(proxy_a, sweep_a, proxy_b, sweep_b)
        fcn.initialize(cache, t1)

        # Compute the TOI on the separating axis. We do this by successively
        # resolving the deepest point. This loop is bounded by the number of vertices.
        done = False
        t2 = t_max
        push_back_iter = 0
        while True:
            # Find the deepest point at t2. Store the witness point indices.
            s2, index_a, index_b = fcn.find_min_separation(t2)

            # Is the final configuration separated?
            if s2 > (target + tolerance):
                # Victory!
                state = SEPARATED
                output_t = t_max
                done = True
                break

            # Has the separation reached tolerance?
            if s2 > (target - tolerance):
                # Advance the sweeps
                t1 = t2
                break

            # Compute the initial separation of the witness points.
            s1 = fcn.evaluate(index_a, index_b, t1)

            # Check for initial overlap. This might happen if the root finder
            # runs out of iterations.
            if s1 < (target - tolerance):
                state = FAILED
                output_t = t1
                done = True
                break

            # Check for touching
            if s1 <= (target + tolerance):
                # Victory! t1 should hold the TOI (could be 0.0).
                state = TOUCHING
                output_t = t1
                done = True
                break

            # Compute 1D root of: f(x) - target = 0
            root_iter_count = 0
            a1 = t1
            a2 = t2
            while True:
                # Use a mix of the secant rule and bisection.
                if root_iter_count & 1:
                    # Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                else:
                    # Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2)

                s = fcn.evaluate(index_a, index_b, t)

                if abs(s - target) < tolerance:
                    # t2 holds a tentative value for t1
                    t2 = t
                    break

                # Ensure we continue to bracket the root.
                if s > target:
                    a1 = t
                    s1 = s
                else:
                    a2 = t
                    s2 = s

                root_iter_count += 1
                toi_root_iters += 1

                if root_iter_count == 50:
                    break

            toi_max_root_iters = max(toi_max_root_iters, root_iter_count)

            push_back_iter += 1

            #if push_back_iter == MAX_POLYGON_VERTICES:
            #    break
            #  TODO: this might be necessary?

        iter_ += 1
        toi_iters += 1

        if done:
            break

        if iter_ == TOI_MAX_ITERS:
            # Root finder got stuck. Semi-victory.
            state = FAILED
            output_t = t1
            break

    toi_max_iters = max(toi_max_iters, iter_)
    return state, output_t
