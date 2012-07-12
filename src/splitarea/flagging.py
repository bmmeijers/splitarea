# -*- coding: utf-8 -*-

from mesher.mesh import Vertex
#from brep.io import as_text, \
#                 geom_from_text, \
#                 as_binary, as_hex, as_text, \
#                 geom_from_binary
#from brep.geometry import LineString, Envelope, Point
from collections import defaultdict
from splitarea.skeleton import SkeletonGraph
from math import atan2, pi

PI2 = 2 * pi

def angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = atan2(dy, dx)
    while angle < 0:
        angle += PI2
    return angle
from math import hypot, degrees

class TriangleVisitor:
    def __init__(self, mesh):
        self.mesh = mesh
        self.triangles = []
        self.segments = []
        self.ext_segments = []
        self.connectors = defaultdict(list)
        self.bridges = defaultdict(list)
        self.visit_interior_without_holes()

    def visit_interior_without_holes(self):
        """All triangles that lie on interior, excluding the ones in the holes
        are collected into self.triangles. A triangle is represented by
        one, arbitrary half edge.
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
        mid_pt = mid_point3(a, b, c)
        pt0 = mid_point2(a, b)
        pt1 = mid_point2(b, c)
        pt2 = mid_point2(c, a)
        assert pt0.flag == 0
        assert pt1.flag == 0
        assert pt2.flag == 0
        self.add_segment( pt0, mid_pt )
        self.add_segment( pt1, mid_pt )
        self.add_segment( pt2, mid_pt )
        # connectors
        self.add_connector(a, pt0)
        self.add_connector(b, pt1)
        self.add_connector(c, pt2)

    def process_1triangle(self, he):
        assert he.constraint
        assert not he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
        # segments
        pt0 = mid_point2(c, a)
        pt1 = mid_point2(b, c)
        # connectors
        self.add_connector(b, pt1)
        self.add_connector(c, pt0)
        #
        assert pt0.flag == 0
        self.add_segment( pt0, pt1 )

    def process_2triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
        pt0 = b
        pt1 = mid_point2(a, c)
        self.add_connector(c, pt1)                        
        # tricky situation, point between two constrained edges should 
        # propagate left / right info (can only happen to type2 and type3 triangles)
        self.add_segment( pt0, pt1 )
    
    def process_3triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
        # segments
        mid_pt = mid_point3(a, b, c)
        #
        self.add_segment( a, mid_pt )
        self.add_segment( b, mid_pt )
        self.add_segment( c, mid_pt )
        # connectors
        # no connectors in this case,
        # all nodes will be connected by definition

    def skeleton_segments(self):
        # create segments using knowledge on internal triangles
        for t in self.triangles:
            # type of triangle (no. of constraints)
            triangle_type = 0
            if t.constraint:
                triangle_type += 1
            if t.next.constraint:
                triangle_type += 2
            if t.next.next.constraint:
                triangle_type += 4
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

    def pick_connectors(self):
        """For nodes that have to be connected we pick the longest connector
        """
        # TODO: alternative: pick connector that has direction closest
        # to half of the angle which the two constrained edges make, 
        # going over the interior of the polygon
        # From the node we can get back to the triangulation: node.he
        # Then rotate around node by taking sibling.next, und so weiter :) 
        for node in self.connectors:
            alternatives = self.connectors[node]
            if len(alternatives) == 1:
                segment = alternatives[0]
                self.segments.append(segment)
            else:
                pt0, pt1, = alternatives[0]
                longest = dist(pt0, pt1)
                segment = (pt0, pt1)
                for alternative in alternatives[1:]:
                    pt0, pt1, = alternative
                    d = dist(pt0, pt1)
                    if d > longest:
                        longest = d
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
                        face_id, cw_id, cw_angle, ccw_id, ccw_angle, = xx
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


                

    def list_table_sg(self):
        print "listing table sg"
        from psycopg2 import connect
        from connect import auth_params
        auth = auth_params()
        connection = connect(host='%s' % auth['host'], 
                                  port=auth['port'], 
                                  database='%s' % auth['database'], 
                                  user='%s' % auth['username'], 
                                  password='%s' % auth['password'])
        cursor = connection.cursor()
        
        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_sg")
        cursor.execute("""
CREATE TABLE tmp_mesh_sg
(
    id int8,
    type text
) WITH OIDS""")
        cursor.execute("""
SELECT AddGeometryColumn('tmp_mesh_sg', 'geometry', -1, 'LINESTRING', 2)
""")
        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_conn;")
        cursor.execute("""
CREATE TABLE tmp_mesh_conn
(
    id int8 NOT NULL,
    type int
) WITH OIDS""")
        cursor.execute("""
SELECT AddGeometryColumn('tmp_mesh_conn', 'geometry', -1, 'LINESTRING', 2)
""")
        connection.commit()
        cursor.close()
        connection.close()
        
    def list_segments_pg(self):
        from psycopg2 import connect
        from connect import auth_params
        auth = auth_params()
        connection = connect(host='%s' % auth['host'], 
                                  port=auth['port'], 
                                  database='%s' % auth['database'], 
                                  user='%s' % auth['username'], 
                                  password='%s' % auth['password'])
        cursor = connection.cursor()        
        for i, pts in enumerate(self.segments):
            pt0, pt1, = pts
            cursor.execute( "INSERT INTO tmp_mesh_sg (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
            i,
            0,
            "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
            ))
        
        for i, pts in enumerate(self.ext_segments):
            pt0, pt1, lft, rgt, = pts
            cursor.execute( "INSERT INTO tmp_mesh_sg (id, type, geometry) VALUES({0}, '{1}', geomfromtext('{2}') );".format(
            i,
            '{0} lft: {1} rgt: {2}'.format(2,lft, rgt),
            "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
            ))
        
        connection.commit()    
        cursor.close()
        connection.close()

    def list_connectors_pg(self):
        from psycopg2 import connect
        from connect import auth_params
        auth = auth_params()
        connection = connect(host='%s' % auth['host'], 
                                  port=auth['port'], 
                                  database='%s' % auth['database'], 
                                  user='%s' % auth['username'], 
                                  password='%s' % auth['password'])
        cursor = connection.cursor()
        i = 10000
        
        for alternatives in self.connectors.itervalues():
            for alternative in alternatives:
                pt0, pt1, = alternative
                command = "INSERT INTO tmp_mesh_conn (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
                i,
                1,
                "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
                )
                cursor.execute( command )
                i += 1
            i += 100
        for alternatives in self.bridges.itervalues():
            for alternative in alternatives:
                pt0, pt1, = alternative
                cursor.execute( "INSERT INTO tmp_mesh_conn (id, type, geometry) VALUES({0}, {1}, geomfromtext('{2}') );".format(
                i,
                2,
                "LINESTRING({0} {1}, {2} {3})".format( pt0.x, pt0.y, pt1.x, pt1.y)
                )
                )
                i += 1
            i += 100
        connection.commit()
        cursor.close()
        connection.close()

    def list_pg(self):
        print "listing mesh to postgres"
        from psycopg2 import connect
        from connect import auth_params
        auth = auth_params()
        connection = connect(host='%s' % auth['host'], 
                                  port=auth['port'], 
                                  database='%s' % auth['database'], 
                                  user='%s' % auth['username'], 
                                  password='%s' % auth['password'])
        cursor = connection.cursor()      
        cursor.execute("DROP TABLE IF EXISTS tmp_mesh_pl;")
        cursor.execute("""
CREATE TABLE tmp_mesh_pl
(
    id int8 UNIQUE NOT NULL,
    flag varchar
) WITH OIDS;""")
        cursor.execute("""
SELECT AddGeometryColumn('tmp_mesh_pl', 'geometry', -1, 'POLYGON', 2);
""")
        print "listing #", len(self.triangles), "triangles"
        for i, t in enumerate(self.triangles):
            a, b, c, = t, t.next, t.next.next
            cursor.execute("""INSERT INTO tmp_mesh_pl (id, flag, geometry) VALUES({0}, {1}, geomfromtext('{2}') );""".format(
                            i,
                            "{0}".format(a.flag),
                            "POLYGON(({0} {1}, {2} {3}, {4} {5}, {6} {7}))".format(                           
                            a.origin.x, a.origin.y, 
                            b.origin.x, b.origin.y, 
                            c.origin.x, c.origin.y, 
                            a.origin.x, a.origin.y)
                            ))
        connection.commit()
        cursor.close()
        connection.close()


class WeightedTriangleVisitor(TriangleVisitor):
#    def __init__(self, mesh):
#        pass
#        raise NotImplementedError('I have never been tested')

    def process_0triangle(self, he):
        assert not he.constraint
        assert not he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
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
        
        elif weight_type in (1, 2, 4): # 1 vertex fixed
            
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

            
        elif weight_type in (3, 5, 6): # 2 vertices fixed
            
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

        elif weight_type == 7: # all vertices fixed
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

    def process_1triangle(self, he):
        assert he.constraint
        assert not he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
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

    def process_2triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert not he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
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


    
    def process_3triangle(self, he):
        assert he.constraint
        assert he.next.constraint
        assert he.next.next.constraint
        a, b, c = he.origin, he.next.origin, he.next.next.origin
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




def mid_point2(pa, pb):
    mid = Vertex( (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0)
    mid.gid = (min(pa.gid, pb.gid), max(pa.gid, pb.gid),)
    return mid

def weighted_mid_point2(pa, pb):
    start = pa
    end = pb
    if len(start.weights) == 1:
        wa = start.weights[0]
    else:
        wa = sum(start.weights) / len(start.weights)

#        raise NotImplementedError('')
    if len(end.weights) == 1:
        wb = end.weights[0]
    else:
        wb = sum(end.weights) / len(end.weights)
#        raise NotImplementedError('')
    
    K = wa / (wa + wb)
    
#    print K, start.weights, end.weights
#    wa = float(min(pa.weights))
#    wb = float(min(pb.weights))

#
#    
#    weights = pa.weights[:]
#    weights.extend(pb.weights)
#    
#    K = wa / sum(weights)
#    

#
##    waa = float(sum(pa.weights))  / len(pa.weights)
##    wba = float(sum(pa.weights))  / len(pb.weights)
##
##    if wa == 0 or wb == 0:
##        K = wa / ( wa + wb )
##    else:
##        K = waa / ( waa + wba )
##        K = min(max(K, 0.02), 0.98)
#    print K, pa.weights, pb.weights
#    
    dx = end.x - start.x
    dy = end.y - start.y
    
    px = start.x + dx * K
    py = start.y + dy * K
    
    v = Vertex(px, py)

    return v 

    
def mid_point3(pa, pb, pc):
    mid = Vertex((pa.x + pb.x + pc.x) / 3.0, (pa.y + pb.y + pc.y) / 3.0 )
    L = [pa.gid, pb.gid, pc.gid]
    L.sort()
    mid.gid = tuple(L) 
    return mid

def weighted_mid_point3(pa, pb, pc):
    a = weighted_mid_point2(pa, pb)
    b = weighted_mid_point2(pb, pc)
    c = weighted_mid_point2(pc, pa)
    v = Vertex((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0 )
    return v


def dist(v0, v1):
    dx = v0.x - v1.x
    dy = v0.y - v1.y
    return hypot(dx, dy)


