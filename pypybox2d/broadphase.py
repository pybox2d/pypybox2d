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

__all__ = ('BroadPhase',)
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

from .dynamictree import DynamicTree

class BroadPhase(object):
    """
    The broad-phase is used for computing pairs and performing volume queries and ray casts.
    This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
    It is up to the client to consume the new pairs and to track subsequent overlap.
    """
    def __init__(self):
        self._tree = DynamicTree() #do_validation=True)

        self._proxy_count = 0
        self._move_buffer = []
        self._pair_buffer = set()

    def create_proxy(self, aabb, user_data):
        """
        Create a proxy with an initial AABB. Pairs are not reported until
        UpdatePairs is called.
        """
        proxy = self._tree.create_proxy(aabb, user_data)
        self._proxy_count += 1
        self._move_buffer.append(proxy)
        return proxy

    def destroy_proxy(self, proxy):
        """Destroy a proxy. It is up to the client to remove any pairs."""
        if proxy in self._move_buffer:
            self._move_buffer.remove(proxy)

        self._proxy_count -= 1
        self._tree.destroy_proxy(proxy)

    def move_proxy(self, proxy, aabb, displacement):
        """
        call move_proxy as many times as you like, then when you are done
        call update_pairs to finalized the proxy pairs (for your time step).
        """
        buffer = self._tree.move_proxy(proxy, aabb, displacement)
        if buffer:
            self._move_buffer.append(proxy)

    def touch_proxy(self, proxy):
        """Call to trigger a re-processing of it's pairs on the next call to UpdatePairs."""
        self._move_buffer.append(proxy)

    def test_overlap(self, proxya, proxyb):
        """Test overlap of fat AABBs."""
        return proxya.aabb.test_overlap(proxyb.aabb)

    @property
    def proxy_count(self):
        """Get the number of proxies."""
        return self._proxy_count

    def update_pairs(self, callback=None):
        """Update the pairs. This yields pairs. This can only add pairs."""
        # Reset pair buffer
        self._pair_buffer.clear()

        # Perform tree queries for all moving proxies
        for proxy in self._move_buffer:
           # We have to query the tree with the fat AABB so that
           # we don't fail to create a pair that may touch later.

           # Query tree, create pairs and add them pair buffer.
           for result in self._tree.query(proxy.aabb):
                if proxy == result:
                    # A proxy cannot form a pair with itself
                    continue
                if (result, proxy) not in self._pair_buffer:
                    self._pair_buffer.add((proxy, result))

        # Reset the move buffer
        self._move_buffer = []

        # And report the pairs back to the client
        for proxy_a, proxy_b in self._pair_buffer:
            if callback:
                callback(proxy_a.user_data, proxy_b.user_data)
            else:
                yield (proxy_a.user_data, proxy_b.user_data)

    def query(self, aabb):
        """
        Query an AABB for overlapping proxies. Each proxy that overlaps 
        the supplied AABB is yielded.
        """
        for result in self._tree.query(aabb):
            yield result

    def ray_cast(self, p1, p2, max_fraction):
        """
        Ray-cast against the proxies in the tree. This relies on the yielded value 
        to perform a exact ray-cast in the case were the proxy contains a shape.
        The yielded value also performs the any collision filtering. This has performance
        roughly equal to k * log(n), where k is the number of collisions and n is the
        number of proxies in the tree.
        Yields a max_fraction and each proxy that is hit by the ray. This is a list:
            [max_fraction, proxy]. To end the ray-cast, you should modify the first entry
            of this list to 0.0. To continue and specify another max_fraction, set it to
            a nonzero value (see tests for examples)
        @param p1 The ray extends from p1 to p1 + max_fraction * (p2 - p1).
        @param p2
        @param max_fraction
        """
        for pair in self._tree.ray_cast(p1, p2, max_fraction):
            yield pair

    @property
    def tree_height(self):
        """Get the height of the embedded tree."""
        return self._tree.height

    @property
    def tree_balance(self):
        """Get the balance of the embedded tree."""
        return self._tree.max_balance

    @property
    def tree_quality(self):
        """Get the quality metric of the embedded tree."""
        return self._tree.area_ratio

if __name__ == '__main__':
    b = BroadPhase()
