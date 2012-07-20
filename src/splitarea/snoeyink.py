# -*- coding: utf-8 -*-
from collections import defaultdict
from math import atan2, pi
from math import hypot

from mesher.mesh import Vertex
from predicates import orient2d

PI2 = 2 * pi

def angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = atan2(dy, dx)
    while angle < 0:
        angle += PI2
    return angle

def mid_point2(pa, pb):
    mid = Vertex( (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0)
    L = [id(pa), id(pb)]
    L.sort()
    mid.gid = tuple(L)
    return mid

def mid_point3(pa, pb, pc):
    A = dist(pb, pc)
    B = dist(pc, pa)
    C = dist(pa, pb)
    # value between 1 and 2
    #
    # 1 = dented junction 
    # 2 = T-junction
    #
    factor = 1.
    a_ = factor
    smallest = A
    b_ = 1.
    c_ = 1.
    if B < smallest:
        a_ = 1.
        b_ = factor
        smallest = B
    if C < smallest:
        a_ = 1.
        b_ = 1.
        c_ = factor
    d_ = a_ + b_ + c_
#        D = a_*A + b_*B + c_*C
    x = (a_*pa.x + b_*pb.x + c_*pc.x) / d_
    y = (a_*pa.y + b_*pb.y + c_*pc.y) / d_
#    mid = Vertex((pa.x + pb.x + pc.x) / 3.0, 
#        (pa.y + pb.y + pc.y) / 3.0 )
    mid = Vertex(x, y)
    L = [id(pa), id(pb), id(pc)]
    L.sort()
    mid.gid = tuple(L)
    return mid

def dist(v0, v1):
    dx = v0.x - v1.x
    dy = v0.y - v1.y
    return hypot(dx, dy)

class TriangleVisitor:
    def __init__(self, mesh):
        self.mesh = mesh
        self.triangles = []
        self.segments = []
        self.ext_segments = []
        self.connectors = defaultdict(list)
        self.bridges = defaultdict(list)
        self.visit_interior_without_holes()
        self.triangle_point = {}

    def visit_interior_without_holes(self):
        """All triangles that lie on interior, excluding the ones in the holes
        are collected into self.triangles. A triangle is represented by
        one, arbitrary halfedge.
        """
        UNKNOWN = 0
        EXTERIOR = 1
        INTERIOR = 2
        self.mesh.reset_flags(UNKNOWN)
        # start at `infinity' point (from large triangle)
        start = None
        walk = self.mesh.locate(self.mesh.vertices[0])
        he = walk[0]
        stack = [he]
        while stack:
            he = stack.pop()
            # flag three he's that form triangle as visited 
            he.flag += EXTERIOR
            he.next.flag += EXTERIOR
            he.next.next.flag += EXTERIOR
            # stack neighboring triangles or break and start `interior' walk
            # first triangle
            if he.sibling is not None and he.sibling.constraint is True:
                start = he.sibling
                break
            elif he.sibling is not None and \
                he.sibling.constraint is False and UNKNOWN == he.sibling.flag:
                stack.append(he.sibling)
            # second triangle
            if he.next.sibling is not None and \
                he.next.sibling.constraint is True:
                start = he.next.sibling
                break
            elif he.next.sibling is not None and \
                he.next.sibling.constraint is False and \
                UNKNOWN == he.next.sibling.flag:
                stack.append(he.next.sibling)
            # third triangle
            if he.next.next.sibling is not None and \
                he.next.next.sibling.constraint is True:
                start = he.next.next.sibling
                break
            elif he.next.next.sibling is not None and \
                he.next.next.sibling.constraint is False and \
                UNKNOWN == he.next.next.sibling.flag:
                stack.append(he.next.next.sibling)
        # interior walk
        assert start is not None
        stack = [start]
        while stack:
            he = stack.pop()
            if he.flag == INTERIOR:
                # already entered via other side, so we skip this time
                continue
            he.flag += INTERIOR
            he.next.flag += INTERIOR
            he.next.next.flag += INTERIOR
            if he.flag == INTERIOR:
                self.triangles.append(he)
            # stack unvisited ones (flag is UNKNOWN), 
            # but do not go over constraints (this leaves out holes)
            if he.sibling.constraint is False and \
                UNKNOWN == he.sibling.flag:
                stack.append(he.sibling)
            if he.next.sibling.constraint is False and \
                UNKNOWN == he.next.sibling.flag:
                stack.append(he.next.sibling)
            if he.next.next.sibling.constraint is False and \
                UNKNOWN == he.next.next.sibling.flag:
                stack.append(he.next.next.sibling)

    def add_segment(self, v0, v1):
        if v0.flag == 3:
            self.bridges[v0].append( (v0, v1) )
        elif v0.flag == 2:
            self.bridges[v0].append( (v0, v1) )
        else:
            assert v0.flag in (0, 1,)
            self.segments.append( (v0, v1) )

    def add_connector(self, start, end):
        if start.flag == 1:
            self.connectors[start].append( (start, end) )
        elif start.flag == 2:
        ## TODO: different for weighted version:
#        if start.flag == 2 and len(start.weights) > 0 and min(start.weights) != 0:
            self.bridges[start].append( (start, end) )
        elif start.flag == 3:
            self.bridges[start].append( (start, end) )

    def process_0triangle(self, he):
        assert not he.constraint
        assert not he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
        # segments
#        mid_pt = mid_point3(a, b, c)
#        pt0 = mid_point2(a, b)
#        pt1 = mid_point2(b, c)
#        pt2 = mid_point2(c, a)
#        assert pt0.flag == 0
#        assert pt1.flag == 0
#        assert pt2.flag == 0
#        self.add_segment( pt0, mid_pt )
#        self.add_segment( pt1, mid_pt )
#        self.add_segment( pt2, mid_pt )
#        # connectors
#        self.add_connector(a, pt0)
#        self.add_connector(b, pt1)
#        self.add_connector(c, pt2)
        
        
        # 1 = dented junction 
        # 2 = T-junction
        #
        factor = 2.
        A = dist(b, c)
        B = dist(c, a)
        C = dist(a, b)
        a_ = factor
        smallest = A
        b_ = 1.
        c_ = 1.
        if B < smallest:
            a_ = 1.
            b_ = factor
            smallest = B
        if C < smallest:
            a_ = 1.
            b_ = 1.
            c_ = factor
        d_ = a_ + b_ + c_
#        D = a_*A + b_*B + c_*C
        x = (a_*a.x + b_*b.x + c_*c.x) / d_
        y = (a_*a.y + b_*b.y + c_*c.y) / d_
        
        fh = open('/tmp/centres.wkt', 'a')
        print >> fh, "POINT({} {});0".format(x, y)
        fh.close()
        
        v = Vertex(x, y)
        self.triangle_point[he] = v 
        self.triangle_point[he.next] = v
        self.triangle_point[he.next.next] = v

    def process_1triangle(self, he):
        assert he.constraint
        assert not he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        # segments
#        pt0 = mid_point2(c, a)
#        pt1 = mid_point2(b, c)
#        # connectors
#        self.add_connector(b, pt1)
#        self.add_connector(c, pt0)
        #
#        assert pt0.flag == 0
#        self.add_segment( pt0, pt1 )
        fh = open('/tmp/centres.wkt', 'a')
        x = (a.x + b.x + c.x * 2.) / 4.
        y = (a.y + b.y + c.y * 2.) / 4.
        
        # Get length of the base of the triangle and its area, this gets the 
        # height of the triangle. 
        # This height is an approximation of the river width at this point
        base = dist(a, b)
        if base:
            height = abs(orient2d(a, b, c)) / base
        else:
            # prevent div by zero
            height = 0.
        print >> fh, "POINT({} {});{}".format(x, y, height)
        
        fh.close()
        v = Vertex(x, y, {'width': round(height * 2.)  / 2.})
        self.triangle_point[he] = v 
        self.triangle_point[he.next] = v
        self.triangle_point[he.next.next] = v

    def process_2triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        pt0 = b
#        pt1 = mid_point2(a, c)
#        self.add_connector(c, pt1)                        
#        # tricky situation, point between two constrained hes should 
#        # propagate left / right info (can only happen to type2 and type3 triangles)
#        self.add_segment( pt0, pt1 )
        
        fh = open('/tmp/centres.wkt', 'a')
        A = dist(b, c)
        B = dist(c, a)
        C = dist(a, b)
#        a_ = 2
#        smallest = A
#        b_ = 1
#        c_ = 1
#        if B < smallest:
#            a_ = 1
#            b_ = 2
#            smallest = B
#        if C < smallest:
#            a_ = 1
#            b_ = 1
#            c_ = 2
        D = A + B + C
#        D = a_*A + b_*B + c_*C
        x = (A/D*a.x + B/D*b.x + C/D*c.x)
        y = (A/D*a.y + B/D*b.y + C/D*c.y)
        print >> fh, "POINT({} {});0".format(x, y)
        fh.close()
        v = Vertex(x, y)
        self.triangle_point[he] = v 
        self.triangle_point[he.next] = v
        self.triangle_point[he.next.next] = v

    def process_3triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert he.next.next.constraint
        return
#        raise NotImplementedError("not yet implemented")
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        # segments
#        mid_pt = mid_point3(a, b, c)
#        #
#        self.add_segment( a, mid_pt )
#        self.add_segment( b, mid_pt )
#        self.add_segment( c, mid_pt )
        # connectors
        # no connectors in this case,
        # all nodes will be connected by definition

    def skeleton_segments(self):
#        fh = open('/tmp/centres.wkt', 'a')
        fh = open('/tmp/triangles.wkt', 'a')
        for t in self.triangles:
            a, b, c = t.origin, t.next.origin, t.next.next.origin
            tri = "POLYGON(({} {}, {} {}, {} {}, {} {}))\n".format(
                a.x, a.y, b.x, b.y, c.x, c.y, a.x, a.y)
            fh.write(tri)
        fh.close()
        # create segments using knowlhe on internal triangles
        for t in self.triangles:
            # type of triangle (no. of constraints)
            triangle_type = 0
            if t.constraint:
                triangle_type += 1
            if t.next.constraint:
                triangle_type += 2
            if t.next.next.constraint:
                triangle_type += 4
                
#            a,b,c, = t.origin, t.next.origin, t.next.next.origin
#            shortest = dist(a, b)
#            pair = (a, b)
#            third = c
#            if dist(b, c) < shortest:
#                shortest = dist(b, c)
#                pair = (b, c)
#                third = a 
#            if dist(c, a) < shortest:
#                shortest = dist(c, a)
#                pair = (c, a)
#                third = b
#            
#            x = (pair[0].x + pair[1].x + third.x * 1.) / 3
#            y = (pair[0].y + pair[1].y + third.y * 1.) / 3
#            print >> fh, "POINT({} {})".format(x, y)
#        fh.close()
            # 0 - triangle
            if triangle_type == 0:
                self.process_0triangle(t)
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                if triangle_type == 1:
                    self.process_1triangle(t)
                elif triangle_type == 2:
                    self.process_1triangle(t.next)                    
                elif triangle_type == 4:
                    self.process_1triangle(t.next.next)
            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                if triangle_type == 3:
                    self.process_2triangle(t)
                elif triangle_type == 5:
                    self.process_2triangle(t.next.next)
                elif triangle_type == 6:
                    self.process_2triangle(t.next)
            # 3 - triangle
            elif triangle_type == 7:
                self.process_3triangle(t)
        
#        fh = open('/tmp/new_segments.wkt', 'a')
#        TPL = "LINESTRING({0.x} {0.y}, {1.x} {1.y})"
        
        def generate(a, b):
            pa = self.triangle_point[a]
            pb = self.triangle_point[b]
            if id(pa) < id(pb):
#                print >> fh, TPL.format(pa, pb)
                self.add_segment(pa, pb)

        for t in self.triangles:
            # type of triangle (no. of constraints)
            triangle_type = 0
            if t.constraint:
                triangle_type += 1
            if t.next.constraint:
                triangle_type += 2
            if t.next.next.constraint:
                triangle_type += 4
            
            try:
                p = self.triangle_point[t]
                a, b, c = t.origin, t.next.origin, t.next.next.origin
                self.add_connector(a, p)
                self.add_connector(b, p)
                self.add_connector(c, p)
            except KeyError:
                print "ERROR: Found triangle of type", triangle_type
            
            if triangle_type == 0:
                generate(t, t.sibling)
                generate(t, t.next.sibling)
                generate(t, t.next.next.sibling)
                
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                
                if triangle_type == 1:
                    generate(t, t.next.sibling)
                    generate(t, t.next.next.sibling)
                    
                elif triangle_type == 2:
                    generate(t, t.sibling)
                    generate(t, t.next.next.sibling)
                        
                elif triangle_type == 4:
                    generate(t, t.sibling)
                    generate(t, t.next.sibling)
            
            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                
                if triangle_type == 3:
#                    print >> fh, TPL.format(self.triangle_point[t], t.next.origin)
                    self.add_segment(self.triangle_point[t], t.next.origin)
                    generate(t, t.next.next.sibling)
                
                elif triangle_type == 5:
#                    print >> fh, TPL.format(self.triangle_point[t], t.origin)
                    self.add_segment(self.triangle_point[t], t.origin)
                    generate(t, t.next.sibling)

                elif triangle_type == 6:
#                    print >> fh, TPL.format(self.triangle_point[t], t.next.next.origin)
                    self.add_segment(self.triangle_point[t], t.next.next.origin)
                    generate(t, t.sibling)
            # 3 - triangle
#            elif triangle_type == 7:
#                TODO: not that important (triangular feature)
#                print self.triangle_point[t]
#        fh.close()

    def pick_connectors(self):
        """For nodes that have to be connected we pick the longest connector
        """
        # TODO: alternative: pick connector that has direction closest
        # to half of the angle which the two constrained hes make, 
        # going over the interior of the polygon
        # From the node we can get back to the triangulation: node.he
        # Then rotate around node by taking sibling.next, und so weiter :)
        
        # longest 
#        for node in self.connectors:
#            alternatives = self.connectors[node]
#            if len(alternatives) == 1:
#                segment = alternatives[0]
#                self.segments.append(segment)
#            else:
#                pt0, pt1, = alternatives[0]
#                longest = dist(pt0, pt1)
#                segment = (pt0, pt1)
#                for alternative in alternatives[1:]:
#                    pt0, pt1, = alternative
#                    d = dist(pt0, pt1)
#                    if d > longest:
#                        longest = d
#                        segment = (pt0, pt1)
#                self.segments.append(segment)
        # shortest 
        for node in self.connectors:
            alternatives = self.connectors[node]
            if len(alternatives) == 1:
                segment = alternatives[0]
                self.segments.append(segment)
            else:
                pt0, pt1, = alternatives[0]
                shortest = dist(pt0, pt1)
                segment = (pt0, pt1)
                for alternative in alternatives[1:]:
                    pt0, pt1, = alternative
                    d = dist(pt0, pt1)
                    if d < shortest:
                        shortest = d
                        segment = (pt0, pt1)
                self.segments.append(segment)

        # special cases --> transfer L/R info into segment graph
        self.ext_segments = []
        for node in self.bridges:
            alternatives = self.bridges[node]
            if node.flag == 3:
                tmp = []
                for alternative in alternatives:
                    start, end = alternative
                    alpha = angle(start, end)
                    for i, xx, in enumerate(start.label):
                        _, _, cw_angle, _, ccw_angle, = xx
                        if cw_angle < alpha and ccw_angle > alpha:
                            break
                    prv = i - 1
                    nxt = (i + 1) % len(start.label)
                    rgt = start.label[prv][0]
                    lft = start.label[nxt][0]
                    tmp.append((start, end, lft, rgt))

                # now check whether multiple bridges are created between two
                # angles (e.g. test case #010) --> split in groups
                
                # this can also be accomplished with itertools groupby
#                things = sorted(tmp, key=lambda x: (x[2], x[3]) )
#                from itertools import groupby
#                for key, group in groupby(things, lambda x: (x[2], x[3])):
#                    for thing in group:
#                        print thing
#                    print " "
                # end of example groupby
                split = defaultdict(list)
                for start, end, lft, rgt in tmp:
                    split[(lft,rgt)].append((start, end))

                for lft, rgt in split:
                    start, end = split[(lft,rgt)][0]
                    self.ext_segments.append((start, end, lft, rgt))
                
            else:
                
                assert node.flag == 2
                if len(alternatives) == 1:
                    segment = alternatives[0]
                    start, end = segment
                    self.ext_segments.append((start, end, node.label, node.label))
                else:
                    v0, v1, = alternatives[0]
                    longest = dist(v0, v1)
                    segment = (v0, v1)
                    for alternative in alternatives[1:]:
                        v0, v1, = alternative
                        d = dist(v0, v1)
                        if d > longest:
                            longest = d
                            segment = alternative
                    start, end = segment
                    self.ext_segments.append((start, end, node.label, node.label))
