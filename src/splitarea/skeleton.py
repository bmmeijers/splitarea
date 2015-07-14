from simplegeom.geometry import LineString
from splitarea.harvest import angle

def coincident(a, b):
    # TODO: make this method epsilon aware
    dx = a.x - b.x
    dy = a.y - b.y
    if abs(dx) == 0. and abs(dy) == 0.:
        return True
    else:
        return False

DEBUG = False

class SkeletonNode(object):

    def __init__(self, vertex_id, external_id, pt):
        self.id = vertex_id
        self.external_id = external_id
        self.geometry = pt

        self.first_edge = None
        self.first_out = None
        self.first_angle = None

        self.degree = 0
        self.label = 0 # for visiting

    def __str__(self):
        if self.id is not None:
            return "N<%s (%s)>" % (self.id, self.geometry)
        else:
            return "N<-- (%s)>" % (self.geometry)

    def remove_edge(self, edge, out):
        """Removes an edge from the skeleton.

        This method is mostly used for unwanted-branch pruning. Pruning these
        branches might lead to artifacts being preserved. Therefore this method
        returns True if there are two edges remaining which are incident at 
        this node. These two edges should be straightened (that is removed and
        an alternative connection should be inserted into the skeleton).
        """
#        print "removing", edge, ", at", self
        ccw_edge, ccw_out, ccw_angle, = self.ccw_next_edge(edge, out)
        cw_edge, cw_out, _, = self.cw_next_edge(edge, out)
        self.degree -= 1
        if self.degree != 0:
            if cw_out:
                cw_edge.lcw = ccw_edge
                cw_edge.lcw_out = ccw_out
            else:
                cw_edge.rcw = ccw_edge
                cw_edge.rcw_out = ccw_out
            if ccw_out:
                ccw_edge.rccw = cw_edge
                ccw_edge.rccw_out = cw_out
            else:
                ccw_edge.lccw = cw_edge
                ccw_edge.lccw_out = cw_out
            if edge is self.first_edge:
                self.first_edge = ccw_edge
                self.first_out = ccw_out
                self.first_angle = ccw_angle
            if cw_edge.external or ccw_edge.external:
                return False
            if cw_edge.unmovable or ccw_edge.unmovable:
                return False
            if cw_edge is not ccw_edge \
                and self.degree == 2 \
                and cw_edge.left_face_id != cw_edge.right_face_id \
                and ccw_edge.left_face_id != ccw_edge.right_face_id \
                and (not cw_edge.external or not ccw_edge.external) \
                and (not cw_edge.unmovable or not ccw_edge.unmovable):
                return True
            else:
                return False
        else:
            self.first_edge = None
            self.first_out = None
            self.first_angle = None
            assert self.degree == 0
            return False

    def add_edge(self, edge, begins):
        if begins:
            assert edge.start_node is self
        else:
            assert edge.end_node is self
        self.degree += 1
        if DEBUG:
            print "adding edge", edge.edge_id, "at", self, "(",self.geometry,")"
        if self.first_edge is None:
            self.first_edge = edge
            self.first_out = begins
            if begins:
                # angle a->b, where a is node (1st vertex) and b is 2nd vertex of line
                new_angle = angle(edge.geometry[0], edge.geometry[1])
                edge.start_angle = new_angle 
            else:
                # angle a->b, where a is node (last vertex) and b is 2nd last vertex of line
                new_angle = angle(edge.geometry[-1], edge.geometry[-2])
                edge.end_angle = new_angle 
            self.first_angle = new_angle
            if begins:
                edge.lcw = edge
                edge.lcw_out = begins
                edge.rccw = edge
                edge.rccw_out = begins
            else:
                edge.lccw = edge
                edge.lccw_out = begins
                edge.rcw = edge
                edge.rcw_out = begins
        else:
            # add edge
            if begins:
                # angle a->b, where a is node (1st vertex) and b is 2nd vertex of line
                new_angle = angle(edge.geometry[0], edge.geometry[1]) 
                edge.start_angle = new_angle 
            else:
                # angle a->b, where a is node (last vertex) and b is 2nd last vertex of line
                new_angle = angle(edge.geometry[-1], edge.geometry[-2]) 
                edge.end_angle = new_angle 

            # loop to find where this edge should be inserted
            prev_edge, prev_out, prev_angle = self.first_edge, self.first_out, self.first_angle
            next_edge, next_out, next_angle = self.ccw_next_edge(prev_edge, prev_out)
            while True:
                if next_edge == self.first_edge and self.first_out == next_out:
                    break

                if new_angle > prev_angle and new_angle < next_angle:
                    break
                prev_edge, prev_out, prev_angle = next_edge, next_out, next_angle
                next_edge, next_out, next_angle = self.ccw_next_edge(prev_edge, prev_out)

#            assert new_angle != next_angle
#            assert new_angle != prev_angle

            # update incident edges their wings, according to found position
            # of new edge

            # next_edge
            if next_out:
                next_edge.rccw = edge # rccw, new
                next_edge.rccw_out = begins
            else:
                next_edge.lccw = edge # lccw, new
                next_edge.lccw_out = begins
            # edge
            if begins:
                edge.lcw = next_edge # lcw, next
                edge.lcw_out = next_out
                edge.rccw = prev_edge # rccw, prev
                edge.rccw_out = prev_out
            else:
                edge.rcw = next_edge # rcw, next
                edge.rcw_out = next_out
                edge.lccw = prev_edge # lccw, prev
                edge.lccw_out = prev_out
            # prev_edge
            if prev_out:
                prev_edge.lcw = edge # lcw, new
                prev_edge.lcw_out = begins
            else:
                prev_edge.rcw = edge # rcw, new
                prev_edge.rcw_out = begins

            # update first edge of this node,
            # if new edge has smallest angle
            if new_angle < self.first_angle:
                self.first_edge = edge
                self.first_out = begins
                self.first_angle = new_angle

        if DEBUG: print " fin adding **", edge

    def ccw_next_edge(self, edge, begins):
        # take next counter clockwise edge at this node
        if begins:
            next_edge = edge.lcw
            next_out = edge.lcw_out
        else:
            next_edge = edge.rcw
            next_out = edge.rcw_out
        # next_angle
        if next_out:
            next_angle = next_edge.start_angle
        else:
            next_angle = next_edge.end_angle
        return (next_edge, next_out, next_angle,)

    def cw_next_edge(self, edge, begins):
        # take next clockwise edge at this node
        if begins:
            next_edge = edge.rccw
            next_out = edge.rccw_out
        else:
            next_edge = edge.lccw
            next_out = edge.lccw_out
        # next_angle
        if next_out:
            next_angle = next_edge.start_angle
        else:
            next_angle = next_edge.end_angle
        return (next_edge, next_out, next_angle,)


class SkeletonEdge(object):
    def __init__(self, 
                 edge_id,
                 start_node, end_node, 
                 left_face_id, right_face_id,
                 geometry, external, unmovable):
        self.edge_id = edge_id
        self.start_node = start_node
        self.end_node = end_node
        self.left_face_id = left_face_id
        self.right_face_id = right_face_id
        self.geometry = geometry
        self.external = external
        self.unmovable = unmovable
        # wings
        # (direction of edge pointed at is kept in <wing>_out)
        #
        #   \     / 
        #    \   /  
        #     \e/   
        # lccw ^ rcw
        #      |  
        #(LF)  |  (RF)
        #      |  
        #  lcw o rccw
        #     /s\ 
        #    /   \ 
        #   /     \
        #
        self.rccw = None
        self.rccw_out = None
        #
        self.rcw = None
        self.rcw_out = None
        #
        self.lccw = None
        self.lccw_out = None
        #
        self.lcw = None
        self.lcw_out = None
        #
        self.label = 0 # for visiting

    def __str__(self):
        if self.lccw_out is None:
            lccw = "?"
            lccw_out = "?"
        else:
            lccw = self.lccw.edge_id
            if self.lccw_out: 
                lccw_out = "+" 
            else: 
                lccw_out = "-"

        if self.rccw_out is None:
            rccw = "?"
            rccw_out = "?"
        else:
            rccw = self.rccw.edge_id
            if self.rccw_out: 
                rccw_out = "+" 
            else: 
                rccw_out = "-"

        if self.lcw_out is None:
            lcw = "?"
            lcw_out = "?"
        else:
            lcw = self.lcw.edge_id
            if self.lcw_out: 
                lcw_out = "+" 
            else: 
                lcw_out = "-"

        if self.rcw_out is None:
            rcw = "?"
            rcw_out = "?"
        else:
            rcw = self.rcw.edge_id
            if self.rcw_out: 
                rcw_out = "+" 
            else: 
                rcw_out = "-"
        return """E<{0}, s{1} e{2}, l{3} r{4}, lccw:{5}{6},rcw:{7}{8} lcw:{9}{10},rccw:{11}{12}, ext:{13}>""".format(
        self.edge_id, 
        self.start_node.id, self.end_node.id, 
        self.left_face_id, self.right_face_id, 
        lccw_out, lccw,
        rcw_out, rcw,
        lcw_out, lcw,
        rccw_out, rccw,
        self.external)

class SkeletonGraph(object):

    def __init__(self):
        self.nodes = {}
        self.faces = {}
        self.ext = set()
        self.edges = [] #set()
        self.new_edges = []
        self.universe_id = None
        self.srid = 0

    def add_node(self, pt, vertex_id, external_id): 
        assert vertex_id is not None
        if vertex_id not in self.nodes:
            n = SkeletonNode(vertex_id, external_id, pt)
            self.nodes[vertex_id] = n
        else:
            n = self.nodes[vertex_id]
        return n

    def add_segment(self, 
                    geometry,
                    # topo structure
                    edge_id = None,
                    start_node_id = None,
                    end_node_id = None,
                    left_face_id = None, 
                    right_face_id = None,
                    # triangulation
                    start_vertex_id = None,
                    end_vertex_id = None, 
                    # extra info
                    external = True,
                    start_external = False, # True if start_vertex_id is None
                    end_external = False, # True if end_vertex_id is None
                    unmovable = False):
        EXTERNAL = 1
        sn = self.add_node(geometry[0], start_vertex_id, start_node_id)
        if DEBUG: print sn
        if start_external:
            sn.label = EXTERNAL
        en = self.add_node(geometry[-1], end_vertex_id, end_node_id)
        try:
            assert coincident(geometry[0], sn.geometry)
        except:
            print geometry[0], sn.geometry
            raise
        try:
            assert coincident(geometry[-1], en.geometry)
        except:
            print geometry[-1], en.geometry 
            raise
        if DEBUG: print en
        if end_external:
            en.label = EXTERNAL
        edge = SkeletonEdge(edge_id, 
                            sn, en, 
                            left_face_id, right_face_id, 
                            geometry, external, unmovable)
        if DEBUG: print "edge", edge.edge_id, "", sn, "", en
        sn.add_edge(edge, begins = True)
        en.add_edge(edge, begins = False)
        self.edges.append(edge) #add(edge)
        if external: # thus external edge
            self.ext.add(edge)

    def remove_node(self, node):
        """Deleting a node from the structure. The node should not be connected
        any more (i.e. degree == 0).
        """
        key = node.id
        assert self.nodes[key].degree == 0
        del self.nodes[key]

    def create_shortcut_edge(self, node):
        edge, out = node.first_edge, node.first_out
        ccw_edge, ccw_out, _, = node.ccw_next_edge(edge, out)
        # get left / right, start / end, geometry for short cut edge
        new_edge_id = edge.edge_id
        if out:
            new_start = edge.end_node
            new_left_face_id = edge.right_face_id
            new_right_face_id = edge.left_face_id
        else:
            new_start = edge.start_node
            new_left_face_id = edge.left_face_id
            new_right_face_id = edge.right_face_id
        if ccw_out:
            new_end = ccw_edge.end_node
        else:
            new_end = ccw_edge.start_node
        geom = LineString(srid=self.srid)
        geom.append(new_start.geometry)
        geom.append(new_end.geometry)
        # remove edge
        self.remove_edge(edge)
        # remove ccw_edge
        self.remove_edge(ccw_edge)
        # insert shortcut
        self.add_segment(
            geom,
            external = False,
            edge_id = new_edge_id,
            start_vertex_id = new_start.id, 
            end_vertex_id = new_end.id,
            left_face_id = new_left_face_id, 
            right_face_id = new_right_face_id,
            start_external = False,
            end_external = False)

    def remove_edge(self, edge):
        """Removing a edge from the Winged Edge structure.
        If node is left with degree = 0 it is also deleted.
        """
        edge.start_node.remove_edge(edge, True) 
        edge.end_node.remove_edge(edge, False)
        self.edges.remove(edge)
        if edge.start_node.degree == 0:
            self.remove_node(edge.start_node)
        edge.start_node = None
        if edge.end_node.degree == 0:
            self.remove_node(edge.end_node)
        edge.end_node = None

    def prune_edge(self, edge):
        """Prunes an edge from the Winged Edge structure.
        
        Pruning is more than removing:
        A new shortcut edge is created, if there are only two internal
        skeleton edges left at a node (those two edges are removed and replaced
        by their shortcut---this avoids a ``jaggy'' skeleton)
        
        If a node is left with degree = 0 it is also deleted.
        """
        shorten_start = edge.start_node.remove_edge(edge, True)
        shorten_end = edge.end_node.remove_edge(edge, False)
        if shorten_start:
            self.create_shortcut_edge(edge.start_node)
        elif edge.start_node.degree == 0:
            self.remove_node(edge.start_node)
        edge.start_node = None
        if shorten_end:
            self.create_shortcut_edge(edge.end_node)
        elif edge.end_node.degree == 0:
            self.remove_node(edge.end_node)
        edge.end_node = None
        self.edges.remove(edge)

    def label_edges(self, value):
        """Set ``value'' to all label properties on all Edges"""
        for edge in self.edges:
            edge.label = value

    def label_nodes(self, value):
        """Set ``value'' to all label properties on all Nodes"""
        for node in self.nodes.itervalues():
            node.label = value

    def label_sides(self):
        """Propagate what is on left and right for given edges onto new segments
        """
        INIT = 0
        LEFT = 1
        RIGHT = 2
        BOTH = LEFT + RIGHT
        self.label_edges(INIT)
        for edge in self.ext:
            if edge.label == BOTH:
                continue
            else:
                first_edge = edge
                while True:
                    # stop if edge is used twice
                    if edge.label == BOTH:
                        break
                    if not edge.label & LEFT:
                        loop_for_face_id = edge.left_face_id
                        visit_left = True
                    elif not edge.label & RIGHT:
                        loop_for_face_id = edge.right_face_id
                        visit_left = False
                    while True: 
                        if visit_left:
                            if not edge.external:
                                edge.left_face_id = loop_for_face_id 
                            elif edge.external and edge.lccw.external:# \
                                #and edge.lccw is edge: #TODO BUG check that this external edge is not the same!!!
                                if edge.lccw_out:
                                    loop_for_face_id = edge.lccw.left_face_id
                                else:
                                    loop_for_face_id = edge.lccw.right_face_id
                            edge.label += LEFT
                            visit_left = edge.lccw_out
                            edge = edge.lccw
                        else:
                            if not edge.external:
                                edge.right_face_id = loop_for_face_id
                            elif edge.external and edge.rccw.external:# \
                                #and edge.rccw is edge:
                                if edge.rccw_out:
                                    loop_for_face_id = edge.rccw.left_face_id
                                else:
                                    loop_for_face_id = edge.rccw.right_face_id
                            edge.label += RIGHT
                            visit_left = edge.rccw_out
                            edge = edge.rccw
                        # check that turning around a node goes well
                        if True:
                            tmp_fid = None
                            if visit_left and edge.left_face_id is not None:
                                tmp_fid = edge.left_face_id
                            elif not visit_left and edge.right_face_id is not None: 
                                tmp_fid = edge.right_face_id
                            if tmp_fid is not None:
                                try:
                                    assert loop_for_face_id == tmp_fid
                                except:
                                    raise
                        #
                        if edge is first_edge:
                            break
        for edge in self.edges:
            if (edge.left_face_id is None and edge.right_face_id is not None) \
                or \
                (edge.right_face_id is None and edge.left_face_id is not None):
                # TODO: skip if adjacent to universe 
                # (we should start on edge that is not zero weight)
                if edge.left_face_id is None and edge.right_face_id == self.universe_id:
                    continue
                if edge.right_face_id is None and edge.left_face_id == self.universe_id:
                    continue
                stack = [edge]
                while stack:
                    edge = stack.pop()
                    if edge.unmovable:
                        continue
                    if edge.left_face_id is None:
                        visit_left = True
                        loop_for_face_id = edge.right_face_id
                    elif edge.right_face_id is None:
                        visit_left = False
                        loop_for_face_id = edge.left_face_id
                    else:
                        continue
                    first_edge = edge
                    while True:
                        if visit_left:
                            assert edge.left_face_id is None
                            edge.left_face_id = loop_for_face_id
                            if edge.right_face_id is None:
                                stack.append(edge)
                            visit_left = edge.lccw_out
                            edge = edge.lccw
                        else:
                            assert edge.right_face_id is None
                            edge.right_face_id = loop_for_face_id
                            if edge.left_face_id is None:
                                stack.append(edge)
                            visit_left = edge.rccw_out
                            edge = edge.rccw
                        if edge is first_edge:
                            break
        error = False
        for edge in self.edges:
            if edge.left_face_id is None or edge.right_face_id is None:
                error = True
        if error:
            raise ValueError('We could not label an edge somehow')

    def prune_branches(self, debug = False):
        """Prune branches, i.e. edges that have same face_id on both sides
        """
        if debug: 
            print "pruning branches"
        removal = set()
        for edge in self.edges:
            if edge.left_face_id == edge.right_face_id:
                removal.add(edge)
        for edge in removal:
            # Prune ('skim') or just remove
            if debug:
                print "removing", edge
            self.remove_edge(edge)
#             self.prune_edge(edge)
        del removal

    def visualize_nodes(self):
        fh = open('/tmp/nodes.wkt', 'w')
        fh.write("nid;geometry\n")
        for node in self.nodes.itervalues():
            print >> fh, node.id, ";", node.geometry
        fh.close()

    def find_new_edges(self, new_node_id = 0, new_edge_id = 0):
        # this method should output new edges, with correct 
        # id, left/right face, start/end node
        # TODO: remove nodes with degree = 2 that do not form loop
        # Q: can such loops exist??
        INIT = 0
        VISITED = 1
        self.new_node_id = new_node_id
        self.new_edge_id = new_edge_id
        if DEBUG:
            print "find_new_edges"
        self.label_edges(INIT)
        # label nodes that do not have an id yet
        for node in self.nodes.itervalues():
            # FIXME: node id generation !!!
            if node.external_id is None:
                self.new_node_id += 1
                node.id = self.new_node_id
        for edge in self.edges:
            if edge.label == VISITED:
                continue
            else:
                if DEBUG: print ""
                start = edge
                edge.label = VISITED
                if DEBUG: group_by = []
                if DEBUG: group_by.append(edge.edge_id)
                sn = edge.start_node
                en = edge.end_node
                lf_id = edge.left_face_id
                rf_id = edge.right_face_id
                check = (min(lf_id,rf_id), max(lf_id,rf_id))
                if DEBUG: print "start:", edge.edge_id, "st:", sn.id, "en:", en.id
                geom = edge.geometry
                STARTSRID = geom.srid
                out = True #e->s, outgoing from s 
                # walk in direction from end -> start
                while True:
                    # break when:
                    # edge next is start
                    # L/R is not same any more
                    # node stepping over has a degree >= 3 or 1 (i.e. !2)
                    if DEBUG: print "now at (sn)", sn
                    if sn.degree != 2:
                        if DEBUG: print "(dir st) break: node.degree != 2 @e", edge.edge_id, "n", sn.id
                        break
#                    if out:
#                        edge = edge.lcw
#                        out = edge.lcw_out
#                    else:
#                        edge = edge.rcw
#                        out = edge.rcw_out
                    next_edge, out, angle = sn.ccw_next_edge(edge, out)
                    if edge.external and next_edge.external:
                        if DEBUG: print "(dir st) external"
                        break
                    edge = next_edge
                    if edge is start:
                        if DEBUG: print "(dir st) break: edge is start", edge.edge_id
                        break
                    if edge.label == VISITED:
                        if DEBUG: print "(dir st) break: edge is already visited", edge.edge_id
                        break
                    if (min(edge.right_face_id, edge.left_face_id), max(edge.right_face_id, edge.left_face_id)) != check:
                        if DEBUG: print "(dir st) break: not same lf / rf"
                        break
                    if out:
                        sn = edge.end_node
                        # use reversed edge.geometry[:-2] for geom
                        # and extend what's there already
                        extend = geom[1:]
                        assert extend.srid == STARTSRID
                        geom = edge.geometry[:]
                        assert geom.srid == STARTSRID
                        geom.reverse()
                        assert geom.srid == STARTSRID
                        geom.extend(extend)
                        assert geom.srid == STARTSRID
#                        geom.extend(edge.geometry[1:]) # correct?
                    else:
                        sn = edge.start_node
                        extend = geom[1:]
                        geom = edge.geometry[:]
                        assert geom.srid == STARTSRID
                        geom.extend(extend)
                        assert geom.srid == STARTSRID
                    if DEBUG: print "(dir st) now at", edge.edge_id, "propagating", lf_id, rf_id
                    if DEBUG: group_by.append(edge.edge_id)
                    edge.label = VISITED
                    

                edge = start
                out = True # s->e,incoming at e (will be flipped in ccw_next_edge
                # walk in direction from start -> end
                while True:
                    if DEBUG: print "now at node (en)", en
                    if DEBUG: print edge
                    if en.degree != 2:
                        if DEBUG: print "(dir en) break: node.degree != 2 @e", edge.edge_id, "n", sn.id, "hoovering at node", sn.id
                        break
#                    edge, out, angle = en.ccw_next_edge(edge, not out)
                    next_edge, out, angle = sn.ccw_next_edge(edge, not out)
                    if edge.external and next_edge.external:
                        if DEBUG: print "(dir e) external"
                        break
                    edge = next_edge
#                    if out:
#                        edge = edge.rcw
#                        out = edge.rcw_out
#                    else:
#                        edge = edge.lcw
#                        out = edge.lcw_out
#                    
                    if edge is start:
                        if DEBUG: print "(dir en) break: edge is start", edge.edge_id
                        break
                    if edge.label == VISITED:
                        if DEBUG: print "(dir en) break: edge is already visited", edge.edge_id
                        break
                    if (min(edge.right_face_id, edge.left_face_id), max(edge.right_face_id, edge.left_face_id)) != check:
                        if DEBUG: print "(dir en) break: not same lf / rf"
                        break
                    if out:
                        en = edge.end_node
                        geom.extend(edge.geometry[1:])
                    else:
                        en = edge.start_node
                        geom.extend(edge.geometry[-2::-1])
                    if DEBUG: print "(dir en) now at", edge.edge_id, "propagating", lf_id, rf_id, "hoovering at node", en.id
                    if DEBUG: group_by.append(edge.edge_id)
                    edge.label = VISITED
                    assert geom.srid == STARTSRID
                if DEBUG: print "fin, group found:", group_by
                try:
                    assert coincident(geom[0], sn.geometry)
                except AssertionError:
                    print geom
                    print sn, en, sn.geometry, en.geometry
                    print sn.geometry, "!=", geom[0]
                    raise
                try:
                    assert coincident(geom[-1], en.geometry)
                except AssertionError:
                    print geom
                    print sn, en, sn.geometry, en.geometry
                    print en.geometry, "!=", geom[-1]
                    raise
                assert sn.id is not None
                assert en.id is not None
#                assert lf_id is not None
#                assert rf_id is not None
                # TODO: skip length calculation
                self.new_edge_id += 1
                #new = (start.edge_id, sn.id, en.id, lf_id, rf_id, st_length(geom), geom)
                new = (self.new_edge_id, sn.id, en.id, lf_id, rf_id, geom.length, geom)
                if DEBUG: print new
                self.new_edges.append(new)
        return

    def unvisited_edge(self, node):
        """Returns edge not yet visited at given ``node''
        """
        VISITED = 1
        edge, out, angle = node.first_edge, node.first_out, node.first_angle
        if edge.label != VISITED and edge.right_face_id is not None and edge.left_face_id is not None:
            return edge, out, angle
        else:
            edge, out, angle, = node.ccw_next_edge(edge, out)
            while edge is not node.first_edge:
                if edge.label != VISITED and edge.right_face_id is not None and edge.left_face_id is not None:
                    break
                edge, out, angle, = node.ccw_next_edge(edge, out)
            return edge, out, angle

    def make_edge_walk(self, edge, out, break_on_external, new_node_id):
        """Visits edges, until node found, which does not have a degree of 2
        """
        if DEBUG: print "i'll break on external edge", break_on_external
        VISITED = 1
        EXTERNAL = 1
        if out:
            start_node = edge.start_node
            end_node = edge.end_node
            left_face_id = edge.left_face_id
            right_face_id = edge.right_face_id
            geom = edge.geometry
        else:
            start_node = edge.end_node
            end_node = edge.start_node
            left_face_id = edge.right_face_id
            right_face_id = edge.left_face_id
            geom = edge.geometry
            geom.reverse()
        start_edge = edge
        edge.label = VISITED
        guard = 0
        if DEBUG: print "starting at", edge
        while True:
            if DEBUG: print "now at", edge
            guard += 1
            if guard > 2500:
                raise ValueError('Too much iteration in make_edge_walk')
            if end_node.degree != 2:
                if DEBUG:  print "degree != 2"
                break
            if break_on_external and end_node.label == EXTERNAL: #
                if DEBUG:  print "external"
                break
            edge, out, angle, = end_node.ccw_next_edge(edge, not out)
            edge.label = VISITED
            if edge is start_edge:
                break
            if out:
                end_node = edge.end_node
                geom.extend(edge.geometry[1:])
            else:
                end_node = edge.start_node
                geom.extend(edge.geometry[-2::-1])
        if DEBUG: print geom[0], start_node.geometry
        if DEBUG: print geom[-1], end_node.geometry
        try:
            assert coincident(geom[0], start_node.geometry)
            assert coincident(geom[-1], end_node.geometry)
        except:
            print start_node.geometry, geom[0], end_node.geometry, geom[-1]
            raise
        assert start_node.id is not None
        assert end_node.id is not None
        assert left_face_id is not None
        assert right_face_id is not None
        assert geom.srid != 0
        self.new_edges.append((edge.edge_id, start_node.id, end_node.id, left_face_id, right_face_id, geom.length, geom))
