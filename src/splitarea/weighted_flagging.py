raise NotImplementedError("This code is not functioning well")

#from cymesher.mesh import Mesh, Vertex
#from brep.io import as_text, \
#                 geom_from_text, \
#                 as_binary, as_hex, as_text, \
#                 geom_from_binary
#from brep.geometry import LineString, Envelope, Point
#from collections import defaultdict
#
#from splitarea.skeleton import SkeletonGraph
#from brep.topomaphe import angle
#from math import hypot
#
#class TriangleVisitor:
#    def __init__(self, mesh):
#        self.mesh = mesh
#        self.triangles = []
#        self.segments = []
#        self.ext_segments = []
#        self.connectors = defaultdict(list)
#        self.bridges = defaultdict(list)
#        self.visit_interior_without_holes()
#
#    def visit_all(self):
#        self.mesh.clear_flags()
#        # start from large triangle
#        walk = self.mesh.locate(self.mesh.points[0])
#        he = walk[0]
#        stack = set([he])
#        while stack:
#            he = stack.pop()
#            self.triangles.append((he.origin, he.next.origin, he.next.next.origin))
#            # flag three he's that form triangle as visited 
#            he.flag = 1
#            he.next.flag = 1
#            he.next.next.flag = 1
#            # stack three neighboring triangles
#            if he.sibling is not None and he.sibling.flag is None:
#                stack.add(he.sibling)
#            if he.next.sibling is not None and he.next.sibling.flag is None:
#                stack.add(he.next.sibling)
#            if he.next.next.sibling is not None and he.next.next.sibling.flag is None:
#                stack.add(he.next.next.sibling)
#
#    def visit_exterior(self):
#        self.mesh.clear_flags()
#        # start from large triangle
#        walk = self.mesh.locate(self.mesh.points[0])
#        he = walk[0]
#        stack = set([he])
#        while stack:
#            he = stack.pop()
#            self.triangles.append((he.origin, he.next.origin, he.next.next.origin))
#            # flag three he's that form triangle as visited 
#            he.flag = 1
#            he.next.flag = 1
#            he.next.next.flag = 1
#            # stack three neighboring triangles
#            if he.sibling is not None and he.sibling.constraint is False and he.sibling.flag is None:
#                stack.add(he.sibling)
#            if he.next.sibling is not None and he.next.sibling.constraint is False and he.next.sibling.flag is None:
#                stack.add(he.next.sibling)
#            if he.next.next.sibling is not None and he.next.next.sibling.constraint is False and he.next.next.sibling.flag is None:
#                stack.add(he.next.next.sibling)
#
#    def visit_outer_boundary(self):
#        """All triangles that share edge with boundary
#        """
#        # self.mesh.clear_flags()
#        UNKNOWN = 0
#        EXTERIOR = 1
#        INTERIOR = 2
#        OUTSIDE = EXTERIOR + INTERIOR
#        for he in self.mesh.half_edges:
#            he.flag = UNKNOWN
#        # start from large triangle
#        walk = self.mesh.locate(self.mesh.points[0])
#        he = walk[0]
#        stack = set([he])
#        while stack:
#            he = stack.pop()
#            # self.triangles.append((he.origin, he.next.origin, he.next.next.origin))
#            # flag three he's that form triangle as visited 
#            he.flag = EXTERIOR
#            he.next.flag = EXTERIOR
#            he.next.next.flag = EXTERIOR
#            # stack three neighboring triangles
#            if he.sibling is not None and he.sibling.constraint is False and (UNKNOWN == he.sibling.flag):
#                stack.add(he.sibling)
#            if he.next.sibling is not None and he.next.sibling.constraint is False and (UNKNOWN == he.next.sibling.flag):
#                stack.add(he.next.sibling)
#            if he.next.next.sibling is not None and he.next.next.sibling.constraint is False and (UNKNOWN == he.next.next.sibling.flag):
#                stack.add(he.next.next.sibling)
#        
#        he = walk[0]       
#        stack = set([he])
#        while stack:
#            he = stack.pop()
#            he.flag += INTERIOR
#            he.next.flag += INTERIOR
#            he.next.next.flag += INTERIOR
#            if he.flag == INTERIOR:
#                self.triangles.append((he.origin, he.next.origin, he.next.next.origin, he.flag))
#            # stack three neighboring triangles
#            if he.flag == OUTSIDE:
#                if he.sibling is not None and he.sibling.flag < OUTSIDE:
#                    stack.add(he.sibling)
#                if he.next.sibling is not None and he.next.sibling.flag < OUTSIDE:
#                    stack.add(he.next.sibling)
#                if he.next.next.sibling is not None and he.next.next.sibling.flag < OUTSIDE:
#                    stack.add(he.next.next.sibling)
#
#    def visit_interior_without_holes(self):
#        """All triangles that lie on interior, excluding the ones in the holes
#        are collected into self.triangles. A triangle is represented by
#        one, arbitrary half edge.
#        """
#        UNKNOWN = 0
#        EXTERIOR = 1
#        INTERIOR = 2
#        self.mesh.reset_flags(UNKNOWN)
#        # start at `infinity' point (from large triangle)
#        walk = self.mesh.locate(self.mesh.vertices[0])
#        he = walk[0]
#        stack = [he]
#        while stack:
#            he = stack.pop()
#            # flag three he's that form triangle as visited 
#            he.flag += EXTERIOR
#            he.next.flag += EXTERIOR
#            he.next.next.flag += EXTERIOR
#            # stack neighboring triangles or break and start `interior' walk
#            # first triangle
#            if he.sibling is not None and he.sibling.constraint is True:
#                start = he.sibling
#                break
#            elif he.sibling is not None and \
#                he.sibling.constraint is False and UNKNOWN == he.sibling.flag:
#                stack.append(he.sibling)
#            # second triangle
#            if he.next.sibling is not None and \
#                he.next.sibling.constraint is True:
#                start = he.next.sibling
#                break
#            elif he.next.sibling is not None and \
#                he.next.sibling.constraint is False and \
#                UNKNOWN == he.next.sibling.flag:
#                stack.append(he.next.sibling)
#            # third triangle
#            if he.next.next.sibling is not None and \
#                he.next.next.sibling.constraint is True:
#                start = he.next.next.sibling
#                break
#            elif he.next.next.sibling is not None and \
#                he.next.next.sibling.constraint is False and \
#                UNKNOWN == he.next.next.sibling.flag:
#                stack.append(he.next.next.sibling)
#        # interior walk
#        stack = [start]
#        while stack:
#            he = stack.pop()
#            if he.flag == INTERIOR:
#                # already entered via other side, so we skip this time
#                continue
#            he.flag += INTERIOR
#            he.next.flag += INTERIOR
#            he.next.next.flag += INTERIOR
#            if he.flag == INTERIOR:
#                self.triangles.append(he)
#            # stack unvisited ones (flag is UNKNOWN), 
#            # but do not go over constraints (this leaves out holes)
#            if he.sibling.constraint is False and \
#                UNKNOWN == he.sibling.flag:
#                stack.append(he.sibling)
#            if he.next.sibling.constraint is False and \
#                UNKNOWN == he.next.sibling.flag:
#                stack.append(he.next.sibling)
#            if he.next.next.sibling.constraint is False and \
#                UNKNOWN == he.next.next.sibling.flag:
#                stack.append(he.next.next.sibling)
#
#    def weighted_skeleton_segments(self):
#        # create segments using knowledge on internal triangles
#        for t in self.triangles:
#            # type of triangle (no. of constraints)
#            triangle_type = 0
#            if t.constraint:
#                triangle_type += 1
#            if t.next.constraint:
#                triangle_type += 2
#            if t.next.next.constraint:
#                triangle_type += 4
#            # 0 - triangle
#            if triangle_type == 0:
#                self.process_w0triangle(t)
#            # 1 - triangle (3 rotations)
#            elif triangle_type in (1, 2, 4):
#                if triangle_type == 1:
#                    self.process_w1triangle(t)
#                elif triangle_type == 2:
#                    self.process_w1triangle(t.next)                    
#                elif triangle_type == 4:
#                    self.process_w1triangle(t.next.next)
#            # 2 - triangle (3 rotations)
#            elif triangle_type in (3, 5, 6):
#                if triangle_type == 3:
#                    self.process_w2triangle(t)
#                elif triangle_type == 5:
#                    self.process_w2triangle(t.next.next)
#                elif triangle_type == 6:
#                    self.process_w2triangle(t.next)
#            # 3 - triangle
#            elif triangle_type == 7:
#                self.process_w3triangle(t)    
#
#    def process_w0triangle(self, he):
#        assert not he.constraint
#        assert not he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        #
#        weight_type = 0
#        if min(a.weights) == 0:
#            weight_type += 1
#        if min(b.weights) == 0:
#            weight_type += 2
#        if min(c.weights) == 0:
#            weight_type += 4
#
#        if weight_type == 0: # no vertices fixed
#            # segments
#            g = weighted_mid_point3(a, b, c)
#            d = weighted_mid_point2(a, b)
#            e = weighted_mid_point2(b, c)
#            f = weighted_mid_point2(c, a)
#            self.add_segment(d, g)
#            self.add_segment(e, g)
#            self.add_segment(f, g)
#            # connectors
#            self.add_connector(a, d)
#            self.add_connector(b, e)
#            self.add_connector(c, f)
#        
#        elif weight_type in (1, 2, 4): # 1 vertex fixed
#            
#            def one_fix(a, b, c):
#                e = weighted_mid_point2(b, c)
#                self.add_segment(a, e)
#                self.add_connector(b, e)
#                self.add_connector(c, a)
#                return
#
#            if weight_type == 1:
#                # a is fixed
#                one_fix(a, b, c)
#            elif weight_type == 2:
#                # b is fixed
#                one_fix(b, c, a)
#            elif weight_type == 4:
#                # c is fixed
#                one_fix(c, b, a)
#
#            
#        elif weight_type in (3, 5, 6): # 2 vertices fixed
#            
#            def two_fix(a, b, c):
#                d = mid_point2(a, b)
#                self.add_segment(a, d)
#                # self.add_segment(b, d) # No dups
#                self.add_connector(c, a)
#                return
#            
#            if weight_type == 3:
#                # a, b is fixed
#                two_fix(a, b, c)
#                pass
#            elif weight_type == 5:
#                # a, c is fixed
#                two_fix(c, a, b)
#            elif weight_type == 6:
#                # c is fixed
#                two_fix(b, c, a)            
#
#        elif weight_type == 7: # all vertices fixed
#            # segments
#            g = mid_point3(a, b, c)
#            d = mid_point2(a, b)
#            e = mid_point2(b, c)
#            f = mid_point2(c, a)
#            self.add_segment(d, g)
#            self.add_segment(e, g)
#            self.add_segment(f, g)
#            # connectors --> no connectors here, will result in dups
#            #self.add_connector(a, d)
#            #self.add_connector(b, e)
#            #self.add_connector(c, f)
#
#    def process_w1triangle(self, he):
#        assert he.constraint
#        assert not he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        weight_type = 0
#        if min(a.weights) == 0:
#            weight_type += 1
#        if min(b.weights) == 0:
#            weight_type += 2
#        if min(c.weights) == 0:
#            weight_type += 4
#
#        if weight_type == 0:
#            # segments
#            f = weighted_mid_point2(c, a)
#            e = weighted_mid_point2(b, c)
#            # connectors
#            self.add_connector(b, e)
#            self.add_connector(c, f)
#            #
#            self.add_segment(e, f)
#        
#        elif weight_type in (1, 2, 4):
#            
#            if weight_type == 1:
#                # a is fixed
#                e = weighted_mid_point2(b, c)
#                self.add_connector(b, e)
#                self.add_connector(c, a)
#                self.add_segment(a, e)
#            
#            elif weight_type == 2:
#                # b is fixed
#                f = weighted_mid_point2(a, c)
#                self.add_connector(c, f)
#                self.add_segment(b, f)
#            
#            elif weight_type == 4:
#                # c is fixed
#                self.add_connector(b, c)
#            
#        elif weight_type in (3, 5, 6):
#                
#            if weight_type == 3:
#                # a, b is fixed
#                self.add_connector(c, a)
#                self.add_segment(a, b)
#                
#            elif weight_type == 5:
#                # a, c is fixed
#                f = mid_point2(a, c)
#                self.add_segment(c, f)
#                self.add_connector(b, c)
#                
#            elif weight_type == 6:
#                # b, c is fixed
#                e = mid_point2(b, c)
#                self.add_connector(b, e)
#                #self.add_segment(c, e) ## overlaps with connector generated?
#                
#        elif weight_type == 7:
#            # segments
#            f = mid_point2(c, a)
#            e = mid_point2(b, c)
#            # connectors
#            self.add_connector(b, e)
#            self.add_connector(c, f)
#            #
#            self.add_segment(a, b)
#            self.add_segment(e, f)
#
#    def process_w2triangle(self, he):
#        assert he.constraint
#        assert he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        weight_type = 0
#        if min(a.weights) == 0:
#            weight_type += 1
#        if min(b.weights) == 0:
#            weight_type += 2
#        if min(c.weights) == 0:
#            weight_type += 4        
#
#        if weight_type == 0:
#        
#            f = weighted_mid_point2(a, c)
#            self.add_connector(c, f)
#            # tricky situation, point between two constrained edges should 
#            # propagate left / right info (can only happen to type2 and type3 triangles)
#            if b.flag == 2:
#                self.bridges[b].append( (b, f) )
#            else:
#                self.add_segment(b, f)
#        
#        elif weight_type in (1, 2, 4):
#            
#            if weight_type == 1:
#                # a is fixed
#                self.add_connector(c, a)
#                if a.flag == 2:
#                    self.bridges[a].append( (a, b) )
#                else:
#                    self.add_segment( a, b)
#
#            elif weight_type == 2:
#                # b is fixed
#                f = weighted_mid_point2(a, c)
#                self.add_connector(c, f)
##                if b.flag == 2:
##                    self.bridges[b].append( (b, f) )
##                else:
#                self.add_segment( b, f)
#                    
#            elif weight_type == 4:
#                # c is fixed
#                if b.flag == 2: # should be b.flag == 2 or c.flag == 2:
#                    self.bridges[b].append( (b, c) )
#                else:
#                    self.add_segment( b, c)
#            
#        elif weight_type in (3, 5, 6):
#
#            if weight_type == 3:
#                # a, b is fixed
#                self.add_connector(c, a)                        
#                # tricky situation, point between two constrained edges should 
#                # propagate left / right info (can only happen to type2 and type3 triangles)
##                if b.flag == 2: # TODO
##                    self.bridges[b].append( (b, a) )
##                else:
#                self.add_segment(b, a)
#
#            elif weight_type == 5:
#                # a, c is fixed
#                f = mid_point2(a, c)
#                self.add_connector(c, f)                        
#                # tricky situation, point between two constrained edges should 
#                # propagate left / right info (can only happen to type2 and type3 triangles)
#                if b.flag == 2:
#                    self.bridges[b].append( (b, f) )
#                else:
#                    self.add_segment(b, f)
#
#            elif weight_type == 6:
#                # b, c is fixed
#                # tricky situation, point between two constrained edges should 
#                # propagate left / right info (can only happen to type2 and type3 triangles)
##                if b.flag == 2: # TODO
##                    self.bridges[b].append( (b, c) )
##                else:
#                self.add_segment(b, c)  
#
#        elif weight_type == 7:
#            f = mid_point2(a, c)
#            self.add_connector(c, f)                        
#            self.add_segment(a, b)
#            self.add_segment(b, c)            
#            # tricky situation, point between two constrained edges should 
#            # propagate left / right info (can only happen to type2 and type3 triangles)
#            if b.flag == 2:
#                self.bridges[b].append( (b, f) )
#            else:
#                self.add_segment(b, f)
#
#
#    
#    def process_w3triangle(self, he):
#        assert he.constraint
#        assert he.next.constraint
#        assert he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        weight_type = 0
#        if min(a.weights) == 0:
#            weight_type += 1
#        if min(b.weights) == 0:
#            weight_type += 2
#        if min(c.weights) == 0:
#            weight_type += 4
#
#        if weight_type == 0:
#            # segments
#            g = weighted_mid_point3(a, b, c)
#            if a.flag == 2:
#                self.bridges[a].append( (a, g) )
#            else:
#                self.add_segment( a, g )
#            if b.flag == 2:
#                self.bridges[b].append( (b, g) )
#            else:
#                self.add_segment( b, g )
#            if c.flag == 2:
#                self.bridges[c].append( (c, g) )
#            else:
#                self.add_segment( c, g )
#            # connectors
#            # no connectors in this case,
#            # all nodes will be connected by definition
#            
#        elif weight_type in (3, 5, 6):
#            def two_fix(a, b, c):
#                g = mid_point2(a, b)
#                if a.flag == 2:
#                    self.bridges[a].append( (a, g) )
#                else:
#                    self.add_segment( a, g )
#                if b.flag == 2:
#                    self.bridges[b].append( (b, g) )
#                else:
#                    self.add_segment( b, g )
#                if c.flag == 2:
#                    self.bridges[c].append( (c, g) )
#                else:
#                    self.add_segment( c, g )
#
#            if weight_type == 3: 
#                # a, b is fixed
#                two_fix(a, b, c)
#            elif weight_type == 5:
#                # a,c is fixed
#                two_fix(c, a, b)
#            elif weight_type == 6:
#                # b, c is fixed
#                two_fix(b, c, a)
#        
#        elif weight_type in (1, 2, 4):
#            raise NotImplementedError("This should not happen (only one 0 weight vertex for 3 triangle)!")
#
#        elif weight_type == 7:
#            self.add_segment(a, b)
#            self.add_segment(b, c)
#            self.add_segment(c, a)
#            # raise NotImplementedError("This should not happen (splittee completely fixed)!")
#
#    def add_segment(self, v0, v1):
#        self.segments.append( (v0, v1) )
#
#    def add_connector(self, start, end):
#        if start.flag == 1:
#            self.connectors[start].append( (start, end) )
#        elif start.flag == 2:
#        ## TODO: different for weighted version:
##        if start.flag == 2 and len(start.weights) > 0 and min(start.weights) != 0:
#            self.bridges[start].append( (start, end) )
#        elif start.flag == 3:
#            print start.label
#            print "angle", angle(end, start), "=", angle(start, end), "segment", start.x, start.y, "-->", end.x, end.y
#            j = -1
#            for i, xx,  in enumerate(start.label):
#                face_id, cw_id, cw_angle, ccw_id, ccw_angle, = xx
#                print i, "--", face_id, cw_id, cw_angle, ccw_id, ccw_angle
#            raise NotImplementedError('dead men walking')        
#
#    def process_0triangle(self, he):
#        assert not he.constraint
#        assert not he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        # segments
#        mid_pt = mid_point3(a, b, c)
#        pt0 = mid_point2(a, b)
#        pt1 = mid_point2(b, c)
#        pt2 = mid_point2(c, a)
#        self.add_segment( pt0, mid_pt )
#        self.add_segment( pt1, mid_pt )
#        self.add_segment( pt2, mid_pt )
#        # connectors
#        self.add_connector(a, pt0)
#        self.add_connector(b, pt1)
#        self.add_connector(c, pt2)
#
#    def process_1triangle(self, he):
#        assert he.constraint
#        assert not he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        # segments
#        pt0 = mid_point2(c, a)
#        pt1 = mid_point2(b, c)
#        # connectors
#        self.add_connector(b, pt1)
#        self.add_connector(c, pt0)
#        #
#        self.add_segment( pt0, pt1 )
#
#    def process_2triangle(self, he):
#        assert he.constraint
#        assert he.next.constraint
#        assert not he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        pt0 = b
#        pt1 = mid_point2(a, c)
#        self.add_connector(c, pt1)                        
#        # tricky situation, point between two constrained edges should 
#        # propagate left / right info (can only happen to type2 and type3 triangles)
#        if pt0.flag == 2:
#            self.bridges[pt0].append( (pt0, pt1) )
#        else:
#            self.add_segment( pt0, pt1 )
#    
#    def process_3triangle(self, he):
#        assert he.constraint
#        assert he.next.constraint
#        assert he.next.next.constraint
#        a, b, c = he.origin, he.next.origin, he.next.next.origin
#        # segments
#        mid_pt = mid_point3(a, b, c)
#        if a.flag == 2:
#            self.bridges[a].append( (a, mid_pt) )
#        else:
#            self.add_segment( a, mid_pt )
#        if b.flag == 2:
#            self.bridges[b].append( (b, mid_pt) )
#        else:
#            self.add_segment( b, mid_pt )
#        if c.flag == 2:
#            self.bridges[c].append( (c, mid_pt) )
#        else:
#            self.add_segment( c, mid_pt )
#        # connectors
#        # no connectors in this case,
#        # all nodes will be connected by definition
#
#    def skeleton_segments(self):
#        # create segments using knowledge on internal triangles
#        for t in self.triangles:
#            # type of triangle (no. of constraints)
#            triangle_type = 0
#            if t.constraint:
#                triangle_type += 1
#            if t.next.constraint:
#                triangle_type += 2
#            if t.next.next.constraint:
#                triangle_type += 4
#            # 0 - triangle
#            if triangle_type == 0:
#                self.process_0triangle(t)
#            # 1 - triangle (3 rotations)
#            elif triangle_type in (1, 2, 4):
#                if triangle_type == 1:
#                    self.process_1triangle(t)
#                elif triangle_type == 2:
#                    self.process_1triangle(t.next)                    
#                elif triangle_type == 4:
#                    self.process_1triangle(t.next.next)
#            # 2 - triangle (3 rotations)
#            elif triangle_type in (3, 5, 6):
#                if triangle_type == 3:
#                    self.process_2triangle(t)
#                elif triangle_type == 5:
#                    self.process_2triangle(t.next.next)
#                elif triangle_type == 6:
#                    self.process_2triangle(t.next)
#            # 3 - triangle
#            elif triangle_type == 7:
#                self.process_3triangle(t)
#
#    def pick_connectors(self):
#        """For nodes that have to be connected we pick the longest connector
#        """
#        # TODO: alternative: pick connector that has direction closest
#        # to half of the angle which the two constrained edges make, 
#        # going over the interior of the polygon
#        # From the node we can get back to the triangulation: node.he
#        # Then rotate around node by taking sibling.next, und so weiter :) 
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
#                
#                self.segments.append(segment)
#
#        self.ext_segments = []
#        for node in self.bridges:
#            alternatives = self.bridges[node]
#            if len(alternatives) == 1:
#                segment = alternatives[0]
#                self.ext_segments.append( segment )
#            else:
#                v0, v1, = alternatives[0]
#                longest = dist(v0, v1)
#                segment = (v0, v1)
#                for alternative in alternatives[1:]:
#                    v0, v1, = alternative
#                    d = dist(v0, v1)
#                    if d > longest:
#                        longest = d
#                        segment = alternative
#                self.ext_segments.append(segment)
#
#    def list_table_sg(self):
#        from psycopg2 import connect
#        
##        print "listing table sg"
#        from connect import auth_params
#        auth = auth_params()
#        connection = connect(host='%s' % auth['host'], 
#                                  port=auth['port'], 
#                                  database='%s' % auth['database'], 
#                                  user='%s' % auth['username'], 
#                                  password='%s' % auth['password'])
#        cursor = connection.cursor()
#        
#        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_sg")
#        cursor.execute("""
#CREATE TABLE tmp_mesh_sg
#(
#    id int8,
#    type int
#) WITH OIDS""")
#        cursor.execute("""
#SELECT AddGeometryColumn('tmp_mesh_sg', 'geometry', -1, 'LINESTRING', 2)
#""")
#        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_conn;")
#        cursor.execute("""
#CREATE TABLE tmp_mesh_conn
#(
#    id int8 NOT NULL,
#    type int
#) WITH OIDS""")
#        cursor.execute("""
#SELECT AddGeometryColumn('tmp_mesh_conn', 'geometry', -1, 'LINESTRING', 2)
#""")
#        connection.commit()
#        cursor.close()
#        connection.close()
#        
#    def list_segments_pg(self):
#        from psycopg2 import connect
#        from connect import auth_params
#        auth = auth_params()
#        connection = connect(host='%s' % auth['host'], 
#                                  port=auth['port'], 
#                                  database='%s' % auth['database'], 
#                                  user='%s' % auth['username'], 
#                                  password='%s' % auth['password'])
#        cursor = connection.cursor()        
#        for i, pts in enumerate(self.segments):
#            pt0, pt1, = pts
#            cursor.execute( "INSERT INTO tmp_mesh_sg (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
#            i,
#            0,
#            "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
#            ))
#        
#        for i, pts in enumerate(self.ext_segments):
#            pt0, pt1, = pts
#            cursor.execute( "INSERT INTO tmp_mesh_sg (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
#            i,
#            2,
#            "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
#            ))
#        
#        connection.commit()    
#        cursor.close()
#        connection.close()
#
#    def list_connectors_pg(self):
#        from psycopg2 import connect
#        from connect import auth_params
#        auth = auth_params()
#        connection = connect(host='%s' % auth['host'], 
#                                  port=auth['port'], 
#                                  database='%s' % auth['database'], 
#                                  user='%s' % auth['username'], 
#                                  password='%s' % auth['password'])
#        cursor = connection.cursor()
#        i = 10000
#        
#        for alternatives in self.connectors.itervalues():
#            for alternative in alternatives:
#                pt0, pt1, = alternative
#                command = "INSERT INTO tmp_mesh_conn (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
#                i,
#                1,
#                "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
#                )
#                cursor.execute( command )
#                i += 1
#            i += 100
#        for alternatives in self.bridges.itervalues():
#            for alternative in alternatives:
#                pt0, pt1, = alternative
#                cursor.execute( "INSERT INTO tmp_mesh_conn (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
#                i,
#                2,
#                "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
#                )
#                )
#                i += 1
#            i += 100
#        connection.commit()
#        cursor.close()
#        connection.close()
#
#    def list_pg(self):
#        print "listing mesh to postgres"
#        from psycopg2 import connect
#        from connect import auth_params
#        auth = auth_params()
#        connection = connect(host='%s' % auth['host'], 
#                                  port=auth['port'], 
#                                  database='%s' % auth['database'], 
#                                  user='%s' % auth['username'], 
#                                  password='%s' % auth['password'])
#        cursor = connection.cursor()      
#        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_pl;")
#        cursor.execute("""
#CREATE TABLE tmp_mesh_pl
#(
#    id int8 UNIQUE NOT NULL,
#    flag varchar
#) WITH OIDS;""")
#        cursor.execute("""
#SELECT AddGeometryColumn('tmp_mesh_pl', 'geometry', -1, 'POLYGON', 2);
#""")
#        print "listing #", len(self.triangles), "triangles"
#        for i, t in enumerate(self.triangles):
#            a, b, c, = t, t.next, t.next.next
#            cursor.execute("""INSERT INTO tmp_mesh_pl (id, flag, geometry) VALUES({0}, {1}, geomfromtext('{2}') );""".format(
#                            i,
#                            "{0}".format(a.flag),
#                            "POLYGON(({0} {1}, {2} {3}, {4} {5}, {6} {7}))".format(                           
#                            a.origin.x, a.origin.y, 
#                            b.origin.x, b.origin.y, 
#                            c.origin.x, c.origin.y, 
#                            a.origin.x, a.origin.y)
#                            ))
#        connection.commit()
#        cursor.close()
#        connection.close()
#
#def mid_point2(pa, pb):
#    #wa = float(min(pa.weights))
#    #wb = float(min(pb.weights))
#    mid = Vertex( (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0)
#    #mid.weights = [min(wa, wb)]
#    return mid
#    
#def mid_point3(pa, pb, pc):
#    #wa = float(min(pa.weights))
#    #wb = float(min(pb.weights))
#    #wc = float(min(pc.weights))
#    mid = Vertex((pa.x + pb.x + pc.x) / 3.0, (pa.y + pb.y + pc.y) / 3.0 )
#    #mid.weights = [min(wa, wb, wc)]
#    return mid
#
#def weighted_mid_point2(pa, pb):
#    wa = float(min(pa.weights))
#    wb = float(min(pb.weights))
#    
#    start = pa
#    end = pb
#
#    waa = float(sum(pa.weights)) # / len(pa.weights)
#    wba = float(sum(pa.weights)) # / len(pb.weights)
#
#    if wa == 0 or wb == 0:
#        K = wa / ( wa + wb )
#    else:
#        K = waa / ( waa + wba )
#        K = min(max(K, 0.02), 0.98)
#
##    K = wa / ( wa + wb )
#
#
##    print wa, wb, K
##    else:
##        end = pa
##        start = pb
##        K = wb / ( wa + wb )
#
#    dx = end.x - start.x
#    dy = end.y - start.y
#    
##    print "*", K,
#    px = start.x + dx * K
#    py = start.y + dy * K
#    v = Vertex(px, py)
#    #v.weights = [min(wa, wb)]
#    return v 
#
#def dist(v0, v1):
#    dx = v0.x - v1.x
#    dy = v0.y - v1.y
#    return hypot(dx, dy)
#
#def weighted_mid_point3(pa, pb, pc):
##    lst = [] 
##    lst.extend(pa.weights)
##    lst.extend(pb.weights)
##    lst.extend(pc.weights)
##    if min(lst) == 1.0:
##        return mid_point3(pa, pb, pc)
#
#    a = weighted_mid_point2(pa, pb)
#    b = weighted_mid_point2(pb, pc)
#    c = weighted_mid_point2(pc, pa)
#    
##    da = dist(a, b)
##    db = dist(b, c)
##    dc = dist(c, a)
##    
##    mn = da
##    mid = 0
##    if db < mn:
##        mn = db
##        mid = 1
##    if dc < mn:
##        mn = dc
##        mid = 2
##    if mid == 0:
##        v = mid_point2(a, b)
##    elif mid == 1:
##        v = mid_point2(b, c)
##    elif mid == 2:
##        v = mid_point2(c, a)
##    
#    v = Vertex((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0 )
#    #v.weights = [mn(a.weights, b.weights, c.weights)]
#    return v
#
#def test():
#    
#    from brep.io import geom_from_text
#    from random import randint, seed
#    
#    wkt = """
#    POLYGON ((1748051.29733583 5136615.82947196 773.937, 1748047.48 5136606.14 774.133, 1748046.06 5136603.88 773.969, 1748042.35 5136597.96 773.537, 1748047.67 5136593.68 773.537, 1748062.48 5136584.98 773.537, 1748067.52 5136582.04 773.537, 1748078.22 5136575.79 773.537, 1748087.88 5136571.84 773.537, 1748100.04 5136568.43 773.537, 1748114.52 5136567.96 773.537, 1748123.7 5136568.71 773.537, 1748126.1 5136568.9 773.537, 1748139.61 5136568.29 773.537, 1748149.57 5136565.95 773.537, 1748151.93 5136555.15 773.537, 1748149.21 5136548.6 773.537, 1748145.32 5136546.63 773.537, 1748142.02 5136544.95 773.537, 1748131.75 5136545.4 773.537, 1748127.61 5136546.26 773.537, 1748103.12 5136551.32 773.537, 1748099.8 5136552.01 773.537, 1748097.94 5136550.26 773.537, 1748104.5 5136544.95 773.537, 1748110.82 5136539.96 773.537, 1748115.85 5136536 773.537, 1748120.75 5136530 773.537, 1748122.84 5136527.43 773.537, 1748130.56 5136525.53 773.529, 1748137.16 5136523.9 773.522, 1748181.68 5136506.95
#773.582, 1748242.81 5136487.67 772.94, 1748259.51 5136493.28 772.94, 1748258.84 5136498.26 772.94, 1748301.42 5136484.25 772.94, 1748297.81 5136473.28 771.782, 1748340.02 5136463.62 771.782, 1748354.35 5136457.53 771.782, 1748368.83 5136450.68 771.782, 1748369.05 5136450.98 771.788, 1748372.99 5136456.65 771.892, 1748377.35 5136461.37 772.002, 1748382.3 5136465.89 772.112, 1748386.44 5136469.71 772.222, 1748392.7 5136473.98 772.332, 1748394.36 5136475.02 772.362, 1748398.85 5136477.82 772.442, 1748404.63 5136480.54 772.552, 1748412.49 5136483.25 772.662, 1748419.53 5136484.98 772.772, 1748426.23 5136486.04 772.882, 1748433.03 5136486.76 772.992, 1748441.07 5136486.75 773.102, 1748448.67 5136486.1 773.212, 1748454.72 5136485.04 773.342, 1748455.57 5136484.81 773.342, 1748455.03 5136493.07 773.342, 1748451.87 5136500.92 773.342, 1748448.06 5136508.98 773.342, 1748443.03 5136515.29 773.342, 1748435.89 5136523.68 773.342, 1748426.88 5136534.43 773.342, 1748419.43 5136547.01 773.
#342, 1748412.45 5136556.31 773.342, 1748403.46 5136567.93 773.342, 1748395.46 5136580.18 773.342, 1748392.94 5136593.95 773.342, 1748395.02 5136601.5 773.342, 1748393.12 5136622.45 773.342, 1748390.53 5136630.67 773.129, 1748384.75 5136630.93 773.129, 1748376.07 5136635.84 773.129, 1748375.44 5136643.04 773.129, 1748369.59 5136648.43 773.129, 1748367.26 5136649.46 773.129, 1748363.05 5136651.32 773.129, 1748358.76 5136653.21 773.129, 1748356.79 5136656.19 773.129, 1748354.07 5136660.32 773.129, 1748348.53 5136665.18 773.129, 1748333.14 5136668.96 773.129, 1748328.14 5136669.42 773.129, 1748320.61 5136670.1 773.129, 1748315.97 5136670.48 773.129, 1748315.3 5136670.54 773.129, 1748304.63 5136671.4 773.129, 1748292.43 5136678.1 773.129, 1748284.63 5136689.93 773.129, 1748282.24 5136693.79 773.129, 1748278.53 5136699.76 773.129, 1748266.66 5136710.23 773.129, 1748257.66 5136711.9 773.129, 1748252.92 5136710.38 773.129, 1748245.92 5136708.12 773.129, 1748233.48 5136705.31 773.129,
#1748223.28 5136703 773.129, 1748211.97 5136693.03 773.129, 1748206.17 5136686 773.114, 1748187.06 5136659.51 773.114, 1748185.87 5136658.15 773.101, 1748181.55 5136653.23 773.054, 1748173.2 5136636.95 773.054, 1748169.79 5136630.29 773.054, 1748157.99 5136613.96 773.054, 1748145.41 5136605.64 773.069, 1748137.18 5136604.48 773.084, 1748133.63 5136604.66 773.084, 1748122.27 5136605.23 773.084, 1748112.05 5136608.23 773.084, 1748096.84 5136610.21 773.084, 1748083.65 5136610.56 773.069, 1748070.37 5136611.7 773.174, 1748054.86 5136614.9 773.384, 1748051.29733583 5136615.82947196 773.937))
#    """
#    
#    poly = geom_from_text(wkt)
#    #if DEBUG: print poly
#    ln = []
#    for ring in poly:
#        for pt in ring:
#            ln.append(pt)
#        
#    ev = poly.envelope
#    #if DEBUG: print ev
#    eps = 10000
#    half_dx = (ev.xmax - ev.xmin) / 2.0
#    dy = (ev.ymax - ev.ymin)
#    # top - middle
#    top_y = ev.ymax + dy + eps
#    top_x = ev.xmin + half_dx
#    # bottom - left
#    left_x = ev.xmin - half_dx - eps
#    left_y = ev.ymin - eps
#    # bottom - right
#    right_x = ev.xmax + half_dx + eps
#    right_y = ev.ymin - eps
#    
#    bnd = [Vertex(left_x,left_y), Vertex(right_x,right_y), Vertex(top_x,top_y)]
#    # return
#    mesh = Mesh(boundary = bnd)
#    prev_pt = None
#    seed("ab")
#    for i, pt in enumerate(ln):
#        pt = Vertex(pt.x, pt.y)
#        if i == 2:
#            pt.flag = 1
#            ext_end = Point(pt.x, pt.y)
#        elif i == 60:
#            pt.flag = 1
#            ext_end2 = Point(pt.x, pt.y)    
#        else:
#            pt.flag = 0 # int(randint(0, 10) in (5, ))
#        mesh.insert(pt)
#        if i > 0:
#            mesh.add_constraint( prev_pt, pt )
#        prev_pt = pt
#
#    wkt_island = """POLYGON((1748250 5136550, 1748260 5136550, 1748260 5136560, 1748250 5136560, 1748250 5136550))
#    """
#    poly = geom_from_text(wkt_island)
#    #if DEBUG: print poly
#    ln = []
#    for ring in poly:
#        for pt in ring:
#            ln.append(pt)
#    for i, pt in enumerate(ln):
#        pt = Vertex(pt.x, pt.y)
#        if i == 2:
#            pt.flag = 2
#            pt.label = 503
#        else:
#            pt.flag = 0 # int(randint(0, 10) in (5, ))
#        mesh.insert(pt)
#        if i > 0:
#            mesh.add_constraint( prev_pt, pt )
#        prev_pt = pt
#
#    visitor = TriangleVisitor(mesh)
#    visitor.skeleton_segments()
#    visitor.pick_connectors()
#
#    visitor.list_table_sg()
##    visitor.list_segments_pg()
#    visitor.list_connectors_pg()
#    visitor.list_pg()
#
##    visualizer = MeshVisualizer(mesh)
##    visualizer.list_pg()
#    
#    # make artificial external edges
#    ext_start = Point(1747940, 5136656)
#    ln = LineString()
#    ln.append(ext_start)
#    ln.append(ext_end)
#    outside_edges = [(500000, ln, 100, True, 101, False, 200, 201)] #(id, geom, sn, en, lf, rf, )
#    
#    ext_start2 = Point(1748550, 5136537)
#    ln = LineString()
#    ln.append(ext_start2)
#    ln.append(ext_end2)
#    outside_edges.append((600000, ln, 200, True, 201, False, 201, 200))
#    
#    skeleton = SkeletonGraph()
#    # first add outside edges
#    for outside_edge in outside_edges:
#        eid, geom, sn, sn_ext, en, en_ext, lf, rf, = outside_edge
#        skeleton.add_segment(geom, 
#                             external = True, 
#                             edge_id = eid, 
#                             left_face_id = lf, 
#                             right_face_id = rf,
#                             start_node_id = sn, 
#                             end_node_id = en,
#                             start_external = sn_ext,
#                             end_external = en_ext
#                             )
#    # add nodes from inner rings ("bridge" connectors)
#    for i, segment in enumerate(visitor.ext_segments):
#        v0, v1, = segment
#        ln = LineString()
#        ln.append(v0.point)
#        ln.append(v1.point)
#        skeleton.add_segment(ln, 
#                             external = True, 
#                             edge_id = i,
#                             left_face_id = v0.label,
#                             right_face_id = v0.label,
##                             start_external = True,
##                             end_external = True
#                             )
#
#    # then add all segments which are unlabeled
#    for i, segment in enumerate(visitor.segments):
#        v0, v1, = segment
#        ln = LineString()
#        ln.append(v0.point)
#        ln.append(v1.point)
#        skeleton.add_segment(ln, 
#                             external = False, 
#                             edge_id = i + len(visitor.ext_segments))
#    skeleton.label_sides()
#    
##    skeleton.prune_branches()
#    
##    skeleton.visualize_nodes()
#    
#    skeleton.find_new_edges()
#    
#
#def main():
#    test()
#
#if __name__ == '__main__':
#    main()
##    load_data()
#    #import cProfile
#    #cProfile.run("main()", "adam.prof")
##    f = load_data
##    cProfile.runctx("f()", globals(), locals())
