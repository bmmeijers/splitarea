#raise NotImplementedError("This code is not functioning well")
from collections import defaultdict, namedtuple
from math import hypot

from tri.delaunay import Vertex

VertexInfo = namedtuple("VertexInfo", "type face_ids vertex_id")

def mid_point2(pa, pb):
    #wa = float(min(pa.weights))
    #wb = float(min(pb.weights))
    mid = Vertex( (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0)
    #mid.weights = [min(wa, wb)]
    return mid

def mid_point3(pa, pb, pc):
    #wa = float(min(pa.weights))
    #wb = float(min(pb.weights))
    #wc = float(min(pc.weights))
    mid = Vertex((pa.x + pb.x + pc.x) / 3.0, (pa.y + pb.y + pc.y) / 3.0 )
    #mid.weights = [min(wa, wb, wc)]
    return mid

def weighted_mid_point2(pa, pb):
    wa = float(min(pa.weights))
    wb = float(min(pb.weights))

    start = pa
    end = pb

    waa = float(sum(pa.weights)) # / len(pa.weights)
    wba = float(sum(pa.weights)) # / len(pb.weights)

    if wa == 0 or wb == 0:
        K = wa / ( wa + wb )
    else:
        K = waa / ( waa + wba )
        #K = min(max(K, 0.02), 0.98)
        K = min(max(K, 0), 1) # make sure we do not go out of bounds

#    K = wa / ( wa + wb )


#    print wa, wb, K
#    else:
#        end = pa
#        start = pb
#        K = wb / ( wa + wb )

    dx = end.x - start.x
    dy = end.y - start.y

#    print "*", K,
    px = start.x + dx * K
    py = start.y + dy * K
    v = Vertex(px, py)
    #v.weights = [min(wa, wb)]
    return v 

def dist(v0, v1):
    dx = v0.x - v1.x
    dy = v0.y - v1.y
    return hypot(dx, dy)

def weighted_mid_point3(pa, pb, pc):
#    lst = [] 
#    lst.extend(pa.weights)
#    lst.extend(pb.weights)
#    lst.extend(pc.weights)
#    if min(lst) == 1.0:
#        return mid_point3(pa, pb, pc)

    a = weighted_mid_point2(pa, pb)
    b = weighted_mid_point2(pb, pc)
    c = weighted_mid_point2(pc, pa)
    
#    da = dist(a, b)
#    db = dist(b, c)
#    dc = dist(c, a)
#    
#    mn = da
#    mid = 0
#    if db < mn:
#        mn = db
#        mid = 1
#    if dc < mn:
#        mn = dc
#        mid = 2
#    if mid == 0:
#        v = mid_point2(a, b)
#    elif mid == 1:
#        v = mid_point2(b, c)
#    elif mid == 2:
#        v = mid_point2(c, a)
#    
    v = Vertex((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0 )
    #v.weights = [mn(a.weights, b.weights, c.weights)]
    return v


class EdgeEdgeWeightedHarvester(object):
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


    def weighted_skeleton_segments(self):
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
                self.process_w0triangle(t)
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                #print "1 triangle"
                if triangle_type == 1:
                    self.process_w1triangle(t, 0)
                elif triangle_type == 2:
                    self.process_w1triangle(t, 1)
                elif triangle_type == 4:
                    self.process_w1triangle(t, 2)
            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                #print "2 triangle"
                if triangle_type == 3: # 1 + 2
                    self.process_w2triangle(t, 0)
                elif triangle_type == 6: # 2 + 4
                    self.process_w2triangle(t, 1)
                elif triangle_type == 5: # 4 + 1
                    self.process_w2triangle(t, 2)
            # 3 - triangle
            elif triangle_type == 7:
                #print "3 triangle"
                self.process_w3triangle(t)

    def process_w0triangle(self, t):
        assert not t.constrained[0]
        assert not t.constrained[1]
        assert not t.constrained[2]
        a, b, c = t.vertices
        #
        weight_type = 0
        if min(a.weights) == 0:
            weight_type += 1
        if min(b.weights) == 0:
            weight_type += 2
        if min(c.weights) == 0:
            weight_type += 4

        if weight_type == 0: # no vertices fixed
            # segments
            g = weighted_mid_point3(a, b, c)
            d = weighted_mid_point2(a, b)
            e = weighted_mid_point2(b, c)
            f = weighted_mid_point2(c, a)
            self.add_segment(d, g)
            self.add_segment(e, g)
            self.add_segment(f, g)
            # connectors
            self.add_connector(a, d)
            self.add_connector(b, e)
            self.add_connector(c, f)
        # 1 vertex fixed
        elif weight_type in (1, 2, 4):
            def one_fix(a, b, c):
                e = weighted_mid_point2(b, c)
                self.add_segment(a, e)
                self.add_connector(b, e)
                self.add_connector(c, a)
                return
            if weight_type == 1:
                # a is fixed
                one_fix(a, b, c)
            elif weight_type == 2:
                # b is fixed
                one_fix(b, c, a)
            elif weight_type == 4:
                # c is fixed
                one_fix(c, b, a)
        # 2 vertices fixed
        elif weight_type in (3, 5, 6):
            def two_fix(a, b, c):
                d = mid_point2(a, b)
                self.add_segment(a, d)
                # self.add_segment(b, d) # No dups
                self.add_connector(c, a)
                return
            if weight_type == 3:
                # a, b is fixed
                two_fix(a, b, c)
                pass
            elif weight_type == 5:
                # a, c is fixed
                two_fix(c, a, b)
            elif weight_type == 6:
                # c is fixed
                two_fix(b, c, a)
        # all vertices fixed
        elif weight_type == 7: 
            # segments
            g = mid_point3(a, b, c)
            d = mid_point2(a, b)
            e = mid_point2(b, c)
            f = mid_point2(c, a)
            self.add_segment(d, g)
            self.add_segment(e, g)
            self.add_segment(f, g)
            # connectors --> no connectors here, will result in dups
            #self.add_connector(a, d)
            #self.add_connector(b, e)
            #self.add_connector(c, f)

    def process_w1triangle(self, t, side):
        assert t.constrained[side]
        assert not t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
#     def process_w1triangle(self, he):
#         assert he.constraint
#         assert not he.next.constraint
#         assert not he.next.next.constraint
#         a, b, c = he.origin, he.next.origin, he.next.next.origin
        weight_type = 0
        if min(a.weights) == 0:
            weight_type += 1
        if min(b.weights) == 0:
            weight_type += 2
        if min(c.weights) == 0:
            weight_type += 4

        if weight_type == 0:
            # segments
            f = weighted_mid_point2(c, a)
            e = weighted_mid_point2(b, c)
            # connectors
            self.add_connector(b, e)
            self.add_connector(c, f)
            #
            self.add_segment(e, f)
        
        elif weight_type in (1, 2, 4):
            
            if weight_type == 1:
                # a is fixed
                e = weighted_mid_point2(b, c)
                self.add_connector(b, e)
                self.add_connector(c, a)
                self.add_segment(a, e)
            
            elif weight_type == 2:
                # b is fixed
                f = weighted_mid_point2(a, c)
                self.add_connector(c, f)
                self.add_segment(b, f)
            
            elif weight_type == 4:
                # c is fixed
                self.add_connector(b, c)
            
        elif weight_type in (3, 5, 6):
                
            if weight_type == 3:
                # a, b is fixed
                self.add_connector(c, a)
                self.add_segment(a, b)
                
            elif weight_type == 5:
                # a, c is fixed
                f = mid_point2(a, c)
                self.add_segment(c, f)
                self.add_connector(b, c)
                
            elif weight_type == 6:
                # b, c is fixed
                e = mid_point2(b, c)
                self.add_connector(b, e)
                #self.add_segment(c, e) ## overlaps with connector generated?
                
        elif weight_type == 7:
            # segments
            f = mid_point2(c, a)
            e = mid_point2(b, c)
            # connectors
            self.add_connector(b, e)
            self.add_connector(c, f)
            #
            self.add_segment(a, b)
            self.add_segment(e, f)

    def process_w2triangle(self, t, side):
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
#     def process_w2triangle(self, he):
#         assert he.constraint
#         assert he.next.constraint
#         assert not he.next.next.constraint
#         a, b, c = he.origin, he.next.origin, he.next.next.origin
        weight_type = 0
        if min(a.weights) == 0:
            weight_type += 1
        if min(b.weights) == 0:
            weight_type += 2
        if min(c.weights) == 0:
            weight_type += 4

        if weight_type == 0:

            f = weighted_mid_point2(a, c)
            self.add_connector(c, f)
            # tricky situation, point between two constrained edges should 
            # propagate left / right info (can only happen to type2 and type3 triangles)
            if b.flag == 2:
                self.bridges[b].append( (b, f) )
            else:
                self.add_segment(b, f)

        elif weight_type in (1, 2, 4):

            if weight_type == 1:
                # a is fixed
                self.add_connector(c, a)
                if a.flag == 2:
                    self.bridges[a].append( (a, b) )
                else:
                    self.add_segment( a, b)
            elif weight_type == 2:
                # b is fixed
                f = weighted_mid_point2(a, c)
                self.add_connector(c, f)
#                if b.flag == 2:
#                    self.bridges[b].append( (b, f) )
#                else:
                self.add_segment( b, f)
            elif weight_type == 4:
                # c is fixed
                if b.flag == 2: # should be b.flag == 2 or c.flag == 2:
                    self.bridges[b].append( (b, c) )
                else:
                    self.add_segment( b, c)

        elif weight_type in (3, 5, 6):

            if weight_type == 3:
                # a, b is fixed
                self.add_connector(c, a)
                # tricky situation, point between two constrained edges should 
                # propagate left / right info (can only happen to type2 and type3 triangles)
#                if b.flag == 2: # TODO
#                    self.bridges[b].append( (b, a) )
#                else:
                self.add_segment(b, a)

            elif weight_type == 5:
                # a, c is fixed
                f = mid_point2(a, c)
                self.add_connector(c, f)
                # tricky situation, point between two constrained edges should 
                # propagate left / right info (can only happen to type2 and type3 triangles)
                if b.flag == 2:
                    self.bridges[b].append( (b, f) )
                else:
                    self.add_segment(b, f)

            elif weight_type == 6:
                # b, c is fixed
                # tricky situation, point between two constrained edges should 
                # propagate left / right info (can only happen to type2 and type3 triangles)
#                if b.flag == 2: # TODO
#                    self.bridges[b].append( (b, c) )
#                else:
                self.add_segment(b, c)

        elif weight_type == 7:
            f = mid_point2(a, c)
            self.add_connector(c, f)
            self.add_segment(a, b)
            self.add_segment(b, c)
            # tricky situation, point between two constrained edges should 
            # propagate left / right info (can only happen to type2 and type3 triangles)
            if b.flag == 2:
                self.bridges[b].append( (b, f) )
            else:
                self.add_segment(b, f)

    def process_w3triangle(self, t):
        side = 0
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
#     def process_w3triangle(self, he):
#         assert he.constraint
#         assert he.next.constraint
#         assert he.next.next.constraint
#         a, b, c = he.origin, he.next.origin, he.next.next.origin
        weight_type = 0
        if min(a.weights) == 0:
            weight_type += 1
        if min(b.weights) == 0:
            weight_type += 2
        if min(c.weights) == 0:
            weight_type += 4

        if weight_type == 0:
            # segments
            g = weighted_mid_point3(a, b, c)
            if a.flag == 2:
                self.bridges[a].append( (a, g) )
            else:
                self.add_segment( a, g )
            if b.flag == 2:
                self.bridges[b].append( (b, g) )
            else:
                self.add_segment( b, g )
            if c.flag == 2:
                self.bridges[c].append( (c, g) )
            else:
                self.add_segment( c, g )
            # connectors
            # no connectors in this case,
            # all nodes will be connected by definition

        elif weight_type in (3, 5, 6):
            def two_fix(a, b, c):
                g = mid_point2(a, b)
                if a.flag == 2: # node.info.type !
                    self.bridges[a].append( (a, g) )
                else:
                    self.add_segment( a, g )
                if b.flag == 2:
                    self.bridges[b].append( (b, g) )
                else:
                    self.add_segment( b, g )
                if c.flag == 2:
                    self.bridges[c].append( (c, g) )
                else:
                    self.add_segment( c, g )

            if weight_type == 3: 
                # a, b is fixed
                two_fix(a, b, c)
            elif weight_type == 5:
                # a,c is fixed
                two_fix(c, a, b)
            elif weight_type == 6:
                # b, c is fixed
                two_fix(b, c, a)

        elif weight_type in (1, 2, 4):
            raise NotImplementedError("This should not happen (only one 0 weight vertex for 3 triangle)!")

        elif weight_type == 7:
            self.add_segment(a, b)
            self.add_segment(b, c)
            self.add_segment(c, a)
            # raise NotImplementedError("This should not happen (splittee completely fixed)!")