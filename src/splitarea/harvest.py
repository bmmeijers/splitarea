# -*- coding: utf-8 -*-
"""Based on a set of triangles that represent the interior of an area
obtain a linear representation of that area

"""

from collections import defaultdict, namedtuple
from math import atan2, pi, hypot

from tri.delaunay import Vertex

VertexInfo = namedtuple("VertexInfo", "type face_ids vertex_id")
# VertexInfo types
# ================
# The types of VertexInfo are quite crucial for correct processing
# See ConnectorPicker and node_to_sectors (tgap.genlib.ops.split)
# for implementation details 
#
# TYPE 0
# Intermediate vertex on an edge, no need to make connection to this
# node
#
# TYPE 1
# node in topology, there needs to be made a connection to this node
# => vertex_id is the node_id of the topology
#
# TYPE 2
# E.g. Hole that needs to be dissolved completely, but for that we need 
# to propagate same label on the whole skeleton!
# => face_ids is integer of the face that forms the hole
#
# TYPE 3
# for touching rings, we need to have a node sector list:
# angles that bound a certain face, so that we can get the correct face
# that overlaps 
# => face_ids is list with 'node sectors', describing which face is valid for 
#    which part around the node (based on start and end angle of that sector) 

PI2 = 2 * pi

def angle(p1, p2):
    """Angle for the segment that goes from p1 to p2, in the range of [0,pi2>"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = atan2(dy, dx)
    while angle < 0:
        angle += PI2
    return angle

def mid_point2(pa, pb):
    """Mid point exactly half way two original points """
    mid = Vertex( (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0)
    L = [id(pa), id(pb)]
    L.sort()
    mid.info = VertexInfo(0, None, tuple(L))
    return mid

def mid_point3(pa, pb, pc, factor = 2):
    """Mid point between 3 original points, the factor parameter
    describes how big the influence of the point opposite of the 
    smallest edge is. """
    A = dist(pb, pc)
    B = dist(pc, pa)
    C = dist(pa, pb)
    # value between 1 and 2
    #
    # 1 = dented junction 
    # 2 = T-junction
    #
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
    mid.info = VertexInfo(0, None, tuple(L))
    return mid

def dist(v0, v1):
    """ cartesian distance """
    dx = v0[0] - v1[0]
    dy = v0[1] - v1[1]
    return hypot(dx, dy)


class EdgeEdgeHarvester(object):
    """ Harvest segments by connecting midpoints that lie on the sides of 
    triangles 

    This harvester makes one pass over the triangles and makes sure not
    to create duplicate geometry, by carefully considering which geometry
    to create (while going ccw around the triangle, only certain connectors
    are created).
    """
    def __init__(self, triangles):
        self.triangles = triangles
        self.segments = []
        self.ext_segments = []
        self.connectors = defaultdict(list)
        self.bridges = defaultdict(list)

    def skeleton_segments(self):
        # create segments using given triangles
        tp = [1, 2, 4]
        for t in self.triangles:
            # type of triangle (we count the number of constraints per triangle)
            triangle_type = 0
            for i, c in enumerate(t.constrained):
                if c:
                    triangle_type += tp[i]
            # 0 - triangle
            if triangle_type == 0:
                #print '0 triangle'
                self.process_0triangle(t)
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                #print "1 triangle"
                if triangle_type == 1:
                    self.process_1triangle(t, 0)
                elif triangle_type == 2:
                    self.process_1triangle(t, 1)
                elif triangle_type == 4:
                    self.process_1triangle(t, 2)
            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                #print "2 triangle"
                if triangle_type == 3: # 1 + 2
                    self.process_2triangle(t, 0)
                elif triangle_type == 6: # 2 + 4
                    self.process_2triangle(t, 1)
                elif triangle_type == 5: # 4 + 1
                    self.process_2triangle(t, 2)
            # 3 - triangle
            elif triangle_type == 7:
                #print "3 triangle"
                self.process_3triangle(t)

    def add_segment(self, v0, v1):
        #print "add_segment", v0, v1
        if v0.info is not None:
            if v0.info.type in (2, 3):
                self.bridges[v0].append( (v0, v1) )
            else:
                #print v0.info.type
                assert v0.info.type in (0, 1,)
                self.segments.append( (v0, v1) )
        else:
            self.segments.append( (v0, v1) )

    def add_connector(self, start, end):
        if start.info is not None:
            if start.info.type in (0, 1):
                #print "add_connector", start, end
                self.connectors[start].append( (start, end) )
            elif start.info.type in (2, 3):
                self.bridges[start].append( (start, end) )
            else:
                assert False, "should not arrive here"
        else:
            self.connectors[start].append( (start, end) )
#         ## TODO: different for weighted version:
# #        if start.flag == 2 and len(start.weights) > 0 and min(start.weights) != 0:
#             self.bridges[start].append( (start, end) )

    def process_0triangle(self, t):
        assert not t.constrained[0]
        assert not t.constrained[1]
        assert not t.constrained[2]
        a, b, c = t.vertices
        # segments
        mid_pt = mid_point3(a, b, c)
        pt0 = mid_point2(a, b)
        pt1 = mid_point2(b, c)
        pt2 = mid_point2(c, a)
#         assert pt0.flag == 0
#         assert pt1.flag == 0
#         assert pt2.flag == 0
        self.add_segment( pt0, mid_pt )
        self.add_segment( pt1, mid_pt )
        self.add_segment( pt2, mid_pt )
        # connectors
        self.add_connector(a, pt0)
        self.add_connector(b, pt1)
        self.add_connector(c, pt2)

    def process_1triangle(self, t, side):
        assert t.constrained[side]
        assert not t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        # Get length of the base of the triangle and its area, this gets the 
        # height of the triangle. 
        # This height is an approximation of the river width at this point
#         base = dist(a, b)
#         if base:
#             height = abs(orient2d(a, b, c)) / base
#         else:
#             # prevent div by zero
#             height = 0.
        #width = round(height * 2.)  / 2.
        # segments
        pt0 = mid_point2(c, a)
        pt1 = mid_point2(b, c)
        #pt0.info = {'width': width}
        #pt1.info = {'width': width}
        # connectors
        self.add_connector(b, pt1)
        self.add_connector(c, pt0)
        self.add_segment( pt0, pt1 )

    def process_2triangle(self, t, side):
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        pt0 = b
        pt1 = mid_point2(a, c)
        self.add_connector(c, pt1)
        # tricky situation, point between two constrained edges should 
        # propagate left / right info (can only happen to type2 and type3 triangles)
        self.add_segment( pt0, pt1 )

    def process_3triangle(self, t):
        side = 0
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        # segments
        mid_pt = mid_point3(a, b, c)
        #
        self.add_segment( a, mid_pt )
        self.add_segment( b, mid_pt )
        self.add_segment( c, mid_pt )
        # connectors
        # no connectors in this case,
        # all nodes will be connected by definition


class MidpointHarvester(object):
    """ Harvest segments by connecting center points of triangles that lie
    on the interior of the triangles 

    This harvester makes two passes over the triangles:
    1. compute per triangle a point that lies inside
    2. connect the points that were found (connections made are based on 
       the neighbouring relationships the triangles have)
    """

    def __init__(self, triangles):
        self.triangles = triangles
        self.segments = []
        self.ext_segments = []
        self.connectors = defaultdict(list)
        self.bridges = defaultdict(list)
        self.triangle_point = {}

    def skeleton_segments(self):
        # generate center points (the dots)
        self.dots()
        # connect the dots generated, and also generate 
        # connections to the skeleton from outside (connectors / bridges)
        self.connect_dots()
 
    def add_segment(self, v0, v1):
        if v0.info is not None:
            if v0.info.type in (2, 3):
                self.bridges[v0].append( (v0, v1) )
            else:
                assert v0.info.type in (0, 1,)
                self.segments.append( (v0, v1) )
        else:
            self.segments.append( (v0, v1) )

    def add_connector(self, start, end):
        if start.info is not None:
            if start.info.type in (0, 1):
                self.connectors[start].append( (start, end) )
            elif start.info.type in (2, 3):
                self.bridges[start].append( (start, end) )
            else:
                assert False, "should not arrive here"
        else:
            self.connectors[start].append( (start, end) )

    def process_0triangle(self, t):
        assert not t.constrained[0]
        assert not t.constrained[1]
        assert not t.constrained[2]
        a, b, c = t.vertices
#         # 1 = dented junction 
#         # 2 = T-junction
#         #
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
        x = (a_*a.x + b_*b.x + c_*c.x) / d_
        y = (a_*a.y + b_*b.y + c_*c.y) / d_
        v = Vertex(x, y)
        v.info = VertexInfo(0, None, (id(v),))
        self.triangle_point[t] = v

    def process_1triangle(self, t, side):
        assert t.constrained[side]
        assert not t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        x = (a.x + b.x + c.x * 2.) / 4.
        y = (a.y + b.y + c.y * 2.) / 4.
        v = Vertex(x, y)
        v.info = VertexInfo(0, None, (id(v),))
        self.triangle_point[t] = v 
 
    def process_2triangle(self, t, side):
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        A = dist(b, c)
        B = dist(c, a)
        C = dist(a, b)
        D = A + B + C
        x = (A/D*a.x + B/D*b.x + C/D*c.x)
        y = (A/D*a.y + B/D*b.y + C/D*c.y)
        v = Vertex(x, y)
        v.info = VertexInfo(0, None, (id(v),))
        self.triangle_point[t] = v 

    def process_3triangle(self, t):
        side = 0
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        # segments
        mid_pt = mid_point3(a, b, c)
        #
        self.add_segment( a, mid_pt )
        self.add_segment( b, mid_pt )
        self.add_segment( c, mid_pt )
        # connectors
        # no connectors in this case,
        # all nodes will be connected by definition
 
    def dots(self):
        # create segments using knowledge on internal triangles
        tp = [1, 2, 4]
        for t in self.triangles:
            # type of triangle (no. of constraints)
            triangle_type = 0
            for i, c in enumerate(t.constrained):
                if c:
                    triangle_type += tp[i]
            # 0 - triangle
            if triangle_type == 0:
                #print '0 triangle'
                self.process_0triangle(t)
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                #print "1 triangle"
                if triangle_type == 1:
                    self.process_1triangle(t, 0)
                elif triangle_type == 2:
                    self.process_1triangle(t, 1)
                elif triangle_type == 4:
                    self.process_1triangle(t, 2)

            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                #print "2 triangle"
                if triangle_type == 3: # 1 + 2
                    self.process_2triangle(t, 0)
                elif triangle_type == 6: # 2 + 4
                    self.process_2triangle(t, 1)
                elif triangle_type == 5: # 4 + 1
                    self.process_2triangle(t, 2)
            # 3 - triangle
            elif triangle_type == 7:
                #print "3 triangle"
                self.process_3triangle(t)
                #raise NotImplementedError("not there yet")
 
    def connect_dots(self):
        # create segments using given triangles
        tp = [1, 2, 4]
        for t in self.triangles:
            # type of triangle (count the number of constraints)
            triangle_type = 0
            for i, c in enumerate(t.constrained):
                if c:
                    triangle_type += tp[i]
            if triangle_type == 0:
                self._connect(t, t.neighbours[0])
                self._connect(t, t.neighbours[1])
                self._connect(t, t.neighbours[2])
                # 3 connectors
                end = self.triangle_point[t]
                for start in t.vertices:
                    self.add_connector(start, end)

            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                if triangle_type == 1:
                    self._connect(t, t.neighbours[1])
                    self._connect(t, t.neighbours[2])
                elif triangle_type == 2:
                    self._connect(t, t.neighbours[0])
                    self._connect(t, t.neighbours[2])
                elif triangle_type == 4:
                    self._connect(t, t.neighbours[0])
                    self._connect(t, t.neighbours[1])
                # 3 connectors
                end = self.triangle_point[t]
                for start in t.vertices:
                    self.add_connector(start, end)

            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                if triangle_type == 3:
                    self._connect(t, t.neighbours[2])
                elif triangle_type == 5:
                    self._connect(t, t.neighbours[1])
                elif triangle_type == 6:
                    self._connect(t, t.neighbours[0])
                # 3 connectors
                end = self.triangle_point[t]
                for start in t.vertices:
                    self.add_connector(start, end)
            # 3 - triangle
            elif triangle_type == 7:
#                 print "3 triangle"
                # nothing to connect, already performed in self.dots()
                pass

    def _connect(self, a, b):
        """ connects 2 neighbouring dots """
        try:
            pa = self.triangle_point[a]
            pb = self.triangle_point[b]
            if id(pa) < id(pb):
#                print >> fh, TPL.format(pa, pb)
                self.add_segment(pa, pb)
        except KeyError:
            print "problem finding triangle"
            pass


class ConnectorPicker(object):
    """Decide on which connectors to use to connect the skeleton to already
    existing edges.
    """
    def __init__(self, harvester):
        self.harvester = harvester

    def pick_connectors(self):
        """For nodes that have to be connected we pick a connector.
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
        for node in self.harvester.connectors:
            alternatives = self.harvester.connectors[node]
            pt0, pt1, = alternatives[0]
            current = dist(pt0, pt1)
            segment = (pt0, pt1)
            for alternative in alternatives[1:]:
                pt0, pt1, = alternative
                d = dist(pt0, pt1)
                if d < current:
                    current = d
                    segment = (pt0, pt1)
            self.harvester.segments.append(segment)
 
        # special cases --> transfer L/R info into segment graph
        for node in self.harvester.bridges:
            alternatives = self.harvester.bridges[node]
            if node.info.type == 3:
                # VERTEX INFO is 3
                # touching ring at a node
                tmp = []
                for alternative in alternatives:
                    start, end = alternative
                    alpha = angle(start, end)
                    for i, xx, in enumerate(start.info.face_ids):
                        _, _, cw_angle, _, ccw_angle, = xx
                        if cw_angle < alpha and ccw_angle > alpha:
                            break
                    else:
                        raise ValueError("could not find proper location based on angles")
                    prv = i - 1
                    nxt = (i + 1) % len(start.info.face_ids)
                    rgt = start.info.face_ids[prv][0]
                    lft = start.info.face_ids[nxt][0]
                    tmp.append((start, end, lft, rgt))
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
                    self.harvester.ext_segments.append((start, end, lft, rgt))
            else:
                # VERTEX INFO is 2
                # this vertex is the start of a ring that is completely 
                # merging in the parent
                # hence, we assure that this connector segment knows what 
                # face is on the inside of the ring
                # node.info.face_ids is one integer, representing a face_id
                assert node.info.type == 2
                v0, v1, = alternatives[0]
                current = dist(v0, v1)
                segment = (v0, v1)
                # in case there are more alternatives, we go over them
                # and pick the longest
                for alternative in alternatives[1:]:
                    v0, v1, = alternative
                    d = dist(v0, v1)
                    if d < current:
                        current = d
                        segment = alternative
                start, end = segment
                self.harvester.ext_segments.append((start, end, 
                                                    node.info.face_ids, 
                                                    node.info.face_ids))
