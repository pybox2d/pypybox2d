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

from common import *
from contact_util import ManifoldPoint, ClipVertex
from copy import copy

def edge_separation(poly1, xf1, edge1, poly2, xf2):
    """Find the separation between poly1 and poly2 for a give edge normal on poly1."""
    vertices1 = poly1._vertices
    vertices2 = poly2._vertices

    normals1 = poly1._normals

    assert(0 <= edge1 < len(vertices1))

    # Convert normal from poly1's frame into poly2's frame.
    normal1_world = xf1._rotation * normals1[edge1]
    normal1 = xf2._rotation.mul_t(normal1_world)

    # Find support vertex on poly2 for -normal.
    dots = [v.dot(normal1) for v in vertices2]
    min_dot = min(dots)
    index = dots.index(min_dot)

    v1 = xf1 * vertices1[edge1]
    v2 = xf2 * vertices2[index]
    separation = (v2 - v1) * normal1_world
    return separation

def max_separation(poly1, xf1, poly2, xf2):
    """
    Find the max separation between poly1 and poly2 using edge normals from poly1.
    @return separation, edge_index
    """
    count1 = len(poly1._vertices)
    normals1 = poly1._normals
    
    # Vector pointing from the centroid of poly1 to the centroid of poly2.
    d = (xf2 * poly2._centroid) - (xf1 * poly1._centroid)
    d_local1 = xf1._rotation.mul_t(d)

    # Find edge normal on poly1 that has the largest projection onto d.
    dots = [normal.dot(d_local1) for normal in normals1]
    max_dot = max(dots)
    edge = dots.index(max_dot)

    # Get the separation for the edge normal.
    s = edge_separation(poly1, xf1, edge, poly2, xf2)

    # Check the separation for the previous edge normal.
    prev_edge = (edge - 1) % count1
    s_prev = edge_separation(poly1, xf1, prev_edge, poly2, xf2)

    # Check the separation for the next edge normal.
    next_edge = (edge + 1) % count1
    s_next = edge_separation(poly1, xf1, next_edge, poly2, xf2)

    # Find the best edge and the search direction.
    if s_prev > s and s_prev > s_next:
        increment = -1
        best_edge = prev_edge
        best_separation = s_prev
    elif s_next > s:
        increment = 1
        best_edge = next_edge
        best_separation = s_next
    else:
        return s, edge

    # Perform a local search for the best edge normal.
    while True:
        if increment == -1:
            edge = (best_edge - 1) % count1
        else:
            edge = (best_edge + 1) % count1

        s = edge_separation(poly1, xf1, edge, poly2, xf2)

        if s > best_separation:
            best_edge = edge
            best_separation = s
        else:
            break

    return best_separation, best_edge
   
def find_incident_edge(poly1, xf1, edge1, poly2, xf2):
    vertices2 = poly2._vertices

    normals1 = poly1._normals
    normals2 = poly2._normals

    assert(0 <= edge1 < len(normals1))

    # Get the normal of the reference edge in poly2's frame.
    normal1 = xf2._rotation.mul_t(xf1._rotation * normals1[edge1])

    # Find the incident edge on poly2.
    dots = [normal1.dot(n) for n in normals2]
    min_dot = min(dots)
    index = dots.index(min_dot)

    # Build the clip vertices for the incident edge.
    i1 = index
    i2 = (i1 + 1) % len(normals2)

    c0=ClipVertex(xf2 * vertices2[i1], edge1, i1, ManifoldPoint.FACE, ManifoldPoint.VERTEX)
    c1=ClipVertex(xf2 * vertices2[i2], edge1, i2, ManifoldPoint.FACE, ManifoldPoint.VERTEX)
    return [c0, c1]

def clip_segment_to_line(v_in, normal, offset, vertex_index_a):
    """
    Sutherland-Hodgman clipping.
    v_in: ClipVertex list
    returns: ClipVertex list
    """
    v_out=[]

    # Calculate the distance of end points to the line
    distance0 = normal.dot(v_in[0].v) - offset
    distance1 = normal.dot(v_in[1].v) - offset

    # If the points are behind the plane
    if distance0 <= 0.0:
        v_out.append(copy(v_in[0]))
    if distance1 <= 0.0:
        v_out.append(copy(v_in[1]))

    # If the points are on different sides of the plane
    if distance0 * distance1 < 0.0:
        # Find intersection point of edge and plane
        interp = distance0 / (distance0 - distance1)

        # VertexA is hitting edgeB.
        cv=ClipVertex(v_in[0].v + interp * (v_in[1].v - v_in[0].v),
                        vertex_index_a, v_in[0].id[1], # id[1] is index B
                        ManifoldPoint.VERTEX, ManifoldPoint.FACE,)
        v_out.append(cv)

    return v_out

