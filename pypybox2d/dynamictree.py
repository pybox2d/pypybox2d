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
__all__ = ('TreeNode', 'DynamicTree',)
__version__ = "$Revision$"
__date__ = "$Date$"
# $Source$

import weakref
from copy import copy
from .common import (Vec2, AABB, scalar_cross, min_vector, max_vector, is_power_of_two, property)
from .settings import (AABB_EXTENSION, AABB_MULTIPLIER, MAX_FLOAT)

class TreeNode(object):
    """A node in the dynamic tree. The client does not interact with this directly."""
    # TODO remove dbg
    __slots__=['__weakref__', 'aabb', 'parent', '_user_data', 'child1', 'child2', 'height',  'dbg']

    def __init__(self):
        self.aabb=AABB()
        self.parent=None
        self._user_data=None
        self.child1=None
        self.child2=None
        # leaf is 0, free node is -1 (height)
        self.height=-1
        self.dbg =-1

    def __repr__(self):
        return 'TreeNode(dbg=%s, aabb=%s)' % (self.dbg, self.aabb)

    @property
    def leaf(self):
        return self.child1==None

    @property
    def user_data(self):
        """Stores, in a weak reference, whatever you want it to."""
        if isinstance(self._user_data, weakref.ReferenceType):
            return self._user_data()
        else:
            return self._user_data
    @user_data.setter
    def user_data(self, value):
        if isinstance(value, weakref.ReferenceType):
            self._user_data = value
        else:
            self._user_data = weakref.ref(value)

    @property
    def next(self):
        """Alias for 'parent', used when in the free node pool"""
        return self.parent
    @next.setter
    def next(self, next_):
        self.parent=next_

class DynamicTree(object):
    """
    A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
    A dynamic tree arranges data in a binary tree to accelerate
    queries such as volume queries and ray casts. Leafs are proxies
    with an AABB. In the tree we expand the proxy AABB by FAT_AABB_FACTOR
    so that the proxy AABB is bigger than the client object. This allows the client
    object to move by small amounts without triggering a tree update.
                                                                                    
    starting_capacity must be a power of 2, and can safely be left at the default
    value of 16.

    Note that in this implementation, as opposed to the C++ version, objects
    are do not need to be relocated upon resizing of the array. Therefore instead
    of dealing with integer indices, we use actual objects.
    """
    def __init__(self, starting_capacity=16, do_validation=False):
        if not is_power_of_two(starting_capacity):
            raise ValueError('starting_capacity must be a power of 2')

        self._do_validation = do_validation

        # Initialize the node pool
        self._root = None
        self._node_capacity = starting_capacity
        self._node_count = 0
        self._nodes = [TreeNode() for i in range(self._node_capacity)]

        # Link the nodes together
        prev_node=self._nodes[0]
        for node in self._nodes[1:]:
            prev_node.next=node
            prev_node=node

        self._nodes[-1].next = None
        self._free_list = self._nodes[0]
        self._insertion_count = 0
        self.new_parent_dbg=0 #TODO dbg

    def create_proxy(self, aabb, user_data, use_weakref=True):
        """
        Create a proxy. Provide a tight fitting AABB and a user_data pointer.
        Optionally uses weakrefs. Specify use_weakref=False to disable them.
        @return the new node (the proxy)
        """
        node = self._allocate_node()
        # Fatten the AABB
        r = Vec2(AABB_EXTENSION, AABB_EXTENSION)
        node.aabb = copy(aabb)
        node.aabb.lower_bound -= r
        node.aabb.upper_bound += r
        node.height = 0
        if use_weakref:
            node._user_data = weakref.ref(user_data)
        else:
            node._user_data = user_data

        self._insert_leaf(node)
        return node

    def destroy_proxy(self, node):
        """
        Destroy a proxy. Ignored if already destroyed by the garbage collector.
        """

        if node is None:
            return
       
        assert(node in self._nodes)

        if not node.leaf:
            raise ValueError('Can only destroy proxies of leaves')

        self._remove_leaf(node)
        self._free_node(node)
    
    def move_proxy(self, node, aabb, displacement):
        """
        Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
        then the proxy is removed from the tree and re-inserted. Otherwise
        the function returns immediately.
        @return true if the proxy was re-inserted.
        """
        assert(node)
        assert(node in self._nodes)
        assert(node.leaf)

        if node.aabb.contains(aabb):
            return False

        self._remove_leaf(node)

        # Extend AABB
        b = copy(aabb)
        r = Vec2(AABB_EXTENSION, AABB_EXTENSION)
        b.lower_bound -= r
        b.upper_bound += r

        # Predict AABB displacement.
        d = AABB_MULTIPLIER * displacement
        if d.x < 0.0:
            b.lower_bound += (d.x, 0)
        else:
            b.upper_bound += (d.x, 0)

        if d.y < 0.0:
            b.lower_bound += (0, d.y)
        else:
            b.upper_bound += (0, d.y)
       
        node.aabb = b
        self._insert_leaf(node)
        return True 

    def query(self, aabb):
        """
        Query an AABB for overlapping proxies. Yields
        each proxy that overlaps the supplied AABB.
        """
        stack=[self._root]
        while stack:
            node=stack.pop()
            if node is None:
                continue
            if node.aabb.test_overlap(aabb):
                if node.leaf:
                    yield node
                else:
                    if node.child1:
                        stack.append(node.child1)
                    if node.child2:
                        stack.append(node.child2)

    def ray_cast(self, p1, p2, max_fraction):
        """
        Ray-cast against the proxies in the tree. This relies on the callback
        to perform a exact ray-cast in the case were the proxy contains a shape.
        The callback also performs the any collision filtering. This has performance
        roughly equal to k * log(n), where k is the number of collisions and n is the
        number of proxies in the tree.
        Yields a max_fraction and each proxy that is hit by the ray. This is a list:
            [max_fraction, proxy]. To end the ray-cast, you should modify the first entry
            of this list to 0.0. To continue and specify another max_fraction, pass in a
            value != 0.0.
        @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        """
        p2 = Vec2(*p2)
        r = p2 - p1
        if r.length_squared <= 0.0:
            raise ValueError('||p2 - p1||^2 > 0.0')
        r.normalize()

        # v is perpendicular to the segment
        v = scalar_cross(1.0, r)
        abs_v = abs(v)

        # Separating axis for segment (Gino, p80).
        # |dot(v, p1 - c)| > dot(|v|, h)
        # Build a bounding box for the segment.
        t = p1 + max_fraction * (p2 - p1)
        segment_aabb=AABB(lower_bound=min_vector(p1, t),
                          upper_bound=max_vector(p1, t))

        stack = [self._root]

        while stack:
            node = stack.pop()
            if node is None:
                continue
            if not node.aabb.test_overlap(segment_aabb):
                continue

            # Separating axis for segment (Gino, p80).
            # |dot(v, p1 - c)| > dot(|v|, h)
            c = node.aabb.center
            h = node.aabb.extents
            separation = abs(v * (p1 -c)) - (abs_v * h)
            if separation > 0.0:
                continue

            if node.leaf:
                pair=[max_fraction, node]
                yield pair

                value = pair[0]
                if value == 0.0:
                    # The client has terminated the ray cast
                    break
                elif value > 0.0:
                    # Update the segment bounding box
                    max_fraction = value
                    t = p1 + max_fraction * (p2 - p1)
                    segment_aabb.lower_bound = min_vector(p1, t)
                    segment_aabb.upper_bound = max_vector(p1, t)

            else:
                stack.append(node.child1)
                stack.append(node.child2)

    @property
    def height(self):
        """
        Cached height. To calculate, use compute_height()
        """
        if self._root is None:
            return 0

        return self._root.height

    @property
    def max_balance(self):
        """
        Get the maximum balance of an node in the tree. The balance is the difference
        in height of the two children of a node.
        """
        max_balance = 0
        nodes = [node for node in self._nodes if node.height > 1]
        for node in nodes:
            assert(not node.leaf)

            balance = abs(node.child2.height - node.child1.height)
            max_balance = max(max_balance, balance)

    @property
    def area_ratio(self):
        """
        Get the ratio of the sum of the node areas to the root area.
        """
        if self._root == None:
            return 0.0

        root_area = self._root.aabb.perimeter
        total_area = 0.0

        # Ignore free nodes in the pool
        areas = [node.aabb.perimeter for node in self._nodes 
                    if node.height >= 0]
        total_area = sum(areas)
        return total_area / root_area

    def rebuild_bottom_up(self):
        """
        Build an optimal tree. Very expensive. For testing.
        """
        nodes = []
        count = 0

        # Build an array of the leaves, and free the rest
        for node in self._nodes:
            if node.height < 0:
                # A free node in the pool
                continue
            if node.leaf:
                node.parent = None
                nodes.append(node)
                count += 1
            else:
                self._free_node(node)

        while count > 1:
            min_cost = MAX_FLOAT
            for node_i in nodes:
                aabb_i = node_i.aabb
                for node_j in nodes:
                    aabb_j = node_j.aabb

                    cost = (aabb_i + aabb_j).perimeter
                    if cost < min_cost:
                        min_i = node_i
                        min_j = node_j
                        min_cost = cost

            child1 = min_i
            child2 = min_j

            parent = self._allocate_node()
            parent.child1 = child1
            parent.child2 = child2
            parent.height = 1 + max(child1.height, child2.height)
            parent.aabb.combine_two(child1.aabb, child2.aabb)
            parent.parent = None

            child1.parent = parent
            child2.parent = parent

            nodes.remove(child1)
            nodes.remove(child2)

            count -= 1

        self._root = nodes[0]

        if self._do_validation:
            self.validate()

    def _allocate_node(self):
        """Allocate a node from the pool. Grow the pool if necessary."""
        if self._free_list==None:
            assert(self._node_count==self._node_capacity)

            # The free list is empty. Add on some additional entries.
            # Double the capacity.
            self._node_capacity *= 2
            new_nodes=[TreeNode() for i in range(self._node_count, self._node_capacity)]
            prev_node=new_nodes[0]
            for node in new_nodes[1:]:
                prev_node.next=node
                prev_node=node
            new_nodes[-1].next = None
            self._nodes.extend(new_nodes)

            self._free_list = new_nodes[0]
        
        # Peel off a node from the free list
        node = self._free_list
        self._free_list = node.next

        node.parent = None
        node.child1 = None
        node.child2 = None
        node.height = 0
        node._user_data = None
        self._node_count += 1
        return node

    def _free_node(self, node):
        """Return a node to the pool."""
        assert(0 < self._node_count)

        node.next = self._free_list
        node.height = -1
        self._free_list = node
        self._node_count -= 1

    def _insert_leaf(self, leaf):
        def descend_cost(child, leaf_AABB, inheritance_cost):
            aabb = leaf_AABB + child.aabb
            if child.leaf:
                _cost = aabb.perimeter + inheritance_cost
            else:
                old_area = child.aabb.perimeter
                new_area = aabb.perimeter
                _cost = (new_area - old_area) + inheritance_cost
            return _cost

        self._insertion_count += 1
        
        if self._root is None:
            self._root = leaf
            leaf.parent = None
            return

        # Find the best sibling for this node
        leaf_AABB = leaf.aabb
        node = self._root
        while not node.leaf:
            area = node.aabb.perimeter

            combined_aabb = node.aabb + leaf_AABB
            combined_area = combined_aabb.perimeter

            # Cost of creating a new parent for this node and the new leaf
            cost = 2.0 * combined_area

            # Minimum cost of pushing the leaf further down the tree
            inheritance_cost = 2.0 * (combined_area - area)

            # Cost of descending into the children
            cost1 = descend_cost(node.child1, leaf_AABB, inheritance_cost)
            cost2 = descend_cost(node.child2, leaf_AABB, inheritance_cost)

            # Descend according to the minimum cost.
            if cost < cost1 and cost < cost2:
                break

            # Descend
            if cost1 < cost2:
                node = node.child1
            else:
                node = node.child2

        sibling = node

        # Create a new parent.
        old_parent = sibling.parent
        new_parent = self._allocate_node()
        new_parent.dbg = chr((ord("a") + self.new_parent_dbg)% 256) # TODO
        self.new_parent_dbg += 1

        new_parent.parent = old_parent
        new_parent._user_data = None
        new_parent.aabb = leaf_AABB + sibling.aabb
        new_parent.height = sibling.height + 1

        if old_parent:
            # The sibling was not the root
            if old_parent.child1 == sibling:
                old_parent.child1 = new_parent
            else:
                old_parent.child2 = new_parent
        else:
            # The sibling was the root
            self._root = new_parent

        new_parent.child1 = sibling
        new_parent.child2 = leaf

        sibling.parent = new_parent
        leaf.parent = new_parent
    
        # Walk back up the tree fixing heights and AABBs
        node = leaf.parent
        while node is not None:
            node = self._balance(node)
            child1 = node.child1
            child2 = node.child2

            assert(child1 is not None)
            assert(child2 is not None)
            node.height = 1 + max(child1.height, child2.height)
            node.aabb.combine_two(child1.aabb, child2.aabb)

            node = node.parent

        if self._do_validation:
            self.validate()

    def _remove_leaf(self, leaf):
        if leaf == self._root:
            self._root = None
            return

        parent = leaf.parent
        if parent.child1 == leaf:
            sibling = parent.child2
        else:
            sibling = parent.child1

        if parent and parent.parent:
            grandparent = parent.parent

            # Destroy parent and connect sibling to grandparent.
            if grandparent.child1 == parent:
                grandparent.child1 = sibling
            else:
                grandparent.child2 = sibling

            sibling.parent = grandparent
            self._free_node(parent)

            # Adjust ancestor bounds
            node = grandparent
            while node:
                node = self._balance(node)
                child1, child2 = node.child1, node.child2
                node.aabb.combine_two(child1.aabb, child2.aabb)
                node.height = 1 + max(child1.height, child2.height)
                
                node = node.parent
        else:
            self._root = sibling
            sibling.parent = None
            self._free_node(parent)
        
        if self._do_validation:
            self.validate()
         
    def _balance(self, node):
        """
        Perform a left or right rotation if node A is imbalanced.
        Returns the new root index.
        """
        assert(node is not None)
        
        if node.leaf or node.height < 2:
            return node

        a = node
        b = node.child1
        c = node.child2

        balance = c.height - b.height
        # Rotate C up
        if balance > 1:
            f = c.child1
            g = c.child2

            # Swap A and C
            c.child1 = a
            c.parent = a.parent
            a.parent = c

            # A's old parent should point to C
            if c.parent is not None:
                if c.parent.child1 == a:
                    c.parent.child1 = c
                else:
                    assert(c.parent.child2 == a)
                    c.parent.child2 = c
            else:
                self._root = c
        
            # Rotate
            if f.height > g.height:
                c.child2 = f
                a.child2 = g
                g.parent = a
                a.aabb.combine_two(b.aabb, g.aabb)
                c.aabb.combine_two(a.aabb, f.aabb)

                a.height = 1 + max(b.height, g.height)
                c.height = 1 + max(a.height, f.height)
            else:
                c.child2 = g
                a.child2 = f
                f.parent = a
                a.aabb.combine_two(b.aabb, f.aabb)
                c.aabb.combine_two(a.aabb, g.aabb)

                a.height = 1 + max(b.height, f.height)
                c.height = 1 + max(a.height, g.height)
            
            return c

        # Rotate B up
        if balance < -1:
            d = b.child1
            e = b.child2

            # Swap A and B
            b.child1 = a
            b.parent = a.parent
            a.parent = b
            
            # A's old parent should point to B
            if b.parent is not None:
                if b.parent.child1 == a:
                    b.parent.child1 = b
                else:
                    assert(b.parent.child2 == a)
                    b.parent.child2 = b
            else:
                self._root = b

            # Rotate
            if d.height > e.height:
                b.child2 = d
                a.child1 = e
                e.parent = a
                a.aabb.combine_two(c.aabb, e.aabb)
                b.aabb.combine_two(a.aabb, d.aabb)

                a.height = 1 + max(c.height, e.height)
                b.height = 1 + max(a.height, d.height)
            
            else:
                b.child2 = e
                a.child1 = d
                d.parent = a
                a.aabb.combine_two(c.aabb, d.aabb)
                b.aabb.combine_two(a.aabb, e.aabb)

                a.height = 1 + max(c.height, d.height)
                b.height = 1 + max(a.height, e.height)
            return b

        return a

    def _compute_height(self, node):
        """Recursively compute height from node"""
        if node is None or node.leaf:
            return 0
        height1 = self._compute_height(node.child1)
        height2 = self._compute_height(node.child2)
        return 1 + max(height1, height2)
    def compute_height(self):
        """Recursively compute height from root node"""
        return self._compute_height(self._root)

    def validate(self):
        """Validate this tree. For testing."""
        self._validate_structure(self._root)
        self._validate_metrics(self._root)
        free_count=0
        free_node = self._free_list
        while free_node:
            free_node=free_node.next
            free_count+=1
        
        assert(self.height == self.compute_height())
        assert(self._node_count + free_count == self._node_capacity)

    def _validate_structure(self, node):
        if node is None:
            return
        elif node == self._root:
            assert(node.parent is None)

        child1, child2 = node.child1, node.child2
        if node.leaf:
            assert(child1 is None)
            assert(child2 is None)
            assert(node.height == 0)
            return
      
        assert(child1.parent == node)
        assert(child2 and child2.parent == node)

        self._validate_structure(child1)
        self._validate_structure(child2)

    def _validate_metrics(self, node):
        if node is None:
            return

        child1, child2 = node.child1, node.child2
        if node.leaf:
            assert(child1 is None)
            assert(child2 is None)
            assert(node.height == 0)
            return

        height1 = child1.height
        height2 = child2.height
        height = 1 + max(height1, height2)

        assert(node.height == height)

        aabb = child1.aabb + child2.aabb
       
        assert(aabb.lower_bound == node.aabb.lower_bound)
        assert(aabb.upper_bound == node.aabb.upper_bound)

        self._validate_metrics(child1)
        self._validate_metrics(child2)

    def _find_depth(self, node, find_depth, depth=0):
        if node is None:
            return []
        if depth == find_depth:
            return [node]

        ret0=self._find_depth(node.child1, find_depth, depth+1)
        ret1=self._find_depth(node.child2, find_depth, depth+1)
        return ret0 + ret1

    def find_nodes_at_depth(self, depth):
        assert(depth >= 0)
        return self._find_depth(self._root, depth)

if __name__=='__main__':
    pass
