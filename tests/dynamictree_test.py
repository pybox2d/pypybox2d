import sys
sys.path.append('..')

import unittest
import random
from random import uniform as rand_float
from pypybox2d import dynamictree
from pypybox2d.common import *
from copy import copy

class Actor(object):
    pass

class random_test(unittest.TestCase):
    def random_aabb(self):
        aabb = AABB(lower_bound=(rand_float(-self.world_extent, self.world_extent), 
                                 rand_float(0.0, 2.0 * self.world_extent)))
        w = (2.0 * self.proxy_extent, 2.0 * self.proxy_extent)
        aabb.upper_bound = aabb.lower_bound + w
        return aabb

    def create_proxy(self):
        if len(self.actors) >= self.max_actors:
            return None

        actor = Actor()
        actor.aabb = self.random_aabb()
        actor.overlap = False
        actor.fraction = 0.0
            
        actor.proxy = self.tree.create_proxy(actor.aabb, actor)
        self.actors.append(actor)
        self.tree.validate()
        actor.proxy.dbg = len(self.actors)
        return actor

    def random_actor(self):
        if not self.actors:
            return None

        for i in range(len(self.actors)):
            actor = random.choice(self.actors)
            if actor.proxy is None:
                continue
            return actor

        return None

    def destroy_proxy(self):
        actor=self.random_actor()
        if actor:
            #print('destroy id %d (%d)' % (actor.proxy.dbg, len(self.actors)))
            self.tree.destroy_proxy(actor.proxy)
            actor.proxy = None
            self.actors.remove(actor)
            return

    def move_aabb(self, aabb):
        d = Vec2(rand_float(-.5, .5), rand_float(-.5, .5)) 
        aabb.lower_bound += d
        aabb.upper_bound += d
        
        c0 = 0.5 * (aabb.lower_bound + aabb.upper_bound)
        min_ = (-self.world_extent, 0.0)
        max_ = (self.world_extent, 2.0 * self.world_extent)
        c = clamp_vector(c0, min_, max_)
        
        aabb.lower_bound += (c - c0)
        aabb.upper_bound += (c - c0)

    def move_proxy(self):
        actor=self.random_actor()
        if actor:
            #print('move', actor.proxy.dbg)
            aabb0 = copy(actor.aabb)
            self.move_aabb(actor.aabb)
            displacement = actor.aabb.center - aabb0.center

            if not self.tree.move_proxy(actor.proxy, actor.aabb, displacement):
                pass
            self.tree.validate()            
            return

    def random_action(self):
        actions = [self.create_proxy, self.destroy_proxy, self.move_proxy]
        do = random.choice(actions)
        do()

    def query(self, aabb):
        #print('--query--')
        for proxy in self.tree.query(aabb):
            actor = proxy.user_data
            actor.overlap = aabb.test_overlap(actor.aabb)
        
        # Ensure that the query resulted in all that it should have
        for actor in self.actors:
            overlap = aabb.test_overlap(actor.aabb)
            assert(overlap == actor.overlap)
        
        #overlapped=len([actor for actor in self.actors if actor.overlap])
        #print('query: overlapped %d / %d' % (overlapped, len(self.actors)))

    def ray_cast(self):
        h = self.world_extent
        p1 = (-5, 5.0 + h)
        p2 = (7, -4.0 + h)
        max_fraction = 1.0

        # Ray cast against the dynamic tree
        for pair in self.tree.ray_cast(p1, p2, max_fraction):
            ret_max_fraction, proxy = pair
            
            actor = proxy.user_data
            hit, normal, fraction = actor.aabb.ray_cast(p1, p2, max_fraction)

            if hit:
                tree_results = [hit, normal, fraction]
                tree_actor = actor
                actor.fraction = fraction
                pair[0] = fraction # pass it back to the raycast function
            else:
                pair[0] = max_fraction # pass it back to the raycast function

        # Then to test it, brute force by ray casting each
        brute_actor = None
        for actor in self.actors:
            if actor.proxy is None:
                continue

            hit, normal, fraction = actor.aabb.ray_cast(p1, p2, max_fraction)
            if hit:
                brute_actor = actor
                brute_results = [hit, normal, fraction]
                max_fraction = fraction

        if brute_actor is not None:
            assert(brute_actor.fraction == tree_actor.fraction)

    def step(self):
        for actor in self.actors:
            actor.fraction = 1.0
            actor.overlap = False

        action_count = max(1, len(self.actors) // 2)
        for i in range(action_count):
            self.random_action()

        self.query(self.query_aabb)
        self.ray_cast()

    def setUp(self):
        self.tree=dynamictree.DynamicTree(16, do_validation=True)
        self.world_extent = 15.0
        self.proxy_extent = 0.5
        self.max_actors = 128

        class Actor(object):
            pass

        self.actors = []
        self.query_aabb=AABB( (-3.0, -4.0 + self.world_extent), (5.0, 6.0 + self.world_extent))
   
    def test_random(self):
        random.seed(0)
        for i in range(100):
            self.step()
        self.tree.height
        self.tree.max_balance
        self.tree.area_ratio

if __name__ ==  '__main__':
    unittest.main()
