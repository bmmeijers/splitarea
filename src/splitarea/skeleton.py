from collections import deque

from simplegeom.geometry import LineString, Point
from topomap.topomap import TopoMap

def make_graph(external, visitor, new_edge_id, universe_id, srid):
    """ Returns a graph representation of the segments and external edges """
    skeleton = TopoMap(universe_id, srid)
    # add outside edges (the external chains)
    for he in external:
        eid, sn, en, lf, rf, geom, = he
        skeleton.add_edge(eid,
                          sn, en,
                          lf, rf,
                          geom, attrs = {'external':True})
    # add segments from inner rings ("bridge" connectors), these segments have
    # the correct face to be propagated
    for i, segment in enumerate(visitor.ext_segments, start = new_edge_id+1):
        v0, v1, lf, rf = segment
        ln = LineString(srid=srid)
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        skeleton.add_edge(i,
                             v0.info.vertex_id, v1.info.vertex_id,
                             lf, rf,
                             ln, attrs = {'external':True}
                         )
        # prevent overlapping edge identifiers
        new_edge_id = i
    # add all segments which do not have a face left/right
    for i, segment in enumerate(visitor.segments, 
                                start = new_edge_id+1):
        v0, v1, = segment
        ln = LineString(srid=srid)
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        start_node_id = v0.info.vertex_id
        if start_node_id is None:
            # note, not an int but tuple to prevent duplicate with external ids
            start_node_id = (id(v0), )
        end_node_id = v1.info.vertex_id
        if end_node_id is None:
            # note, not an int but tuple to prevent duplicate with external ids
            end_node_id = (id(v1), ) 
        skeleton.add_edge(i,
                             start_node_id,
                             end_node_id,
                             None, None,
                             ln,
                             attrs = {'external':False})
    return skeleton, new_edge_id

def label_sides(skeleton):
    """ Sets the correct faces for both sides of the edges,
    starting from external edges """
    # find external edges
    external = set()
    for he in skeleton.half_edges.itervalues():
        if he.attrs['external']:
            external.add(he)
            external.add(he.twin)
    # propagate the face id on edges that are not external edges
    for he in external:
        face = he.face
        while he.next.attrs['external'] == False:
            he = he.next
            # the face id currently should be None
            assert he.face.id is None 
            # connect edge to face of external edge
            he.face = face 
    # FIXME:
    # with zero weights there can be cycles!
    # postcondition: edge has face on both sides
#     for he in skeleton.half_edges.itervalues():
#         assert he.face.id is not None, he.id
#         assert he.twin.face.id is not None, he.id
#    # FIXME: universe_id !!
#    # FIXME: this should result in all cycles because of 0 weight edges
#    # also being labeled
#     for he in skeleton.half_edges.itervalues():
#         if he.face is None or he.twin.face is None:
#             # (we should start on edge that is not zero weight)
#             if (he.face is None and he.twin.face.id == skeleton.universe_id) or \
#                 (he.twin.face is None and he.face.id == skeleton.universe_id):
#                 continue
#             stack = [he]
#             while stack:
#                 he = stack.pop()
#                 if he.face is not None:
#                     loop_for_face = he.twin.face # take face at other side
#                     start = he
#                 elif he.twin.face is not None:
#                     loop_for_face = he.face # take face at other side
#                     start = he.twin
#                 else:
#                     # edge was seen earlier
#                     continue
#                 he = start
#                 while True:
#                     he = he.next
#                     assert he.face is None
#                     he.face = loop_for_face
#                     if he.twin is None:
#                         stack.append(he.twin)
#                     if he is start:
#                         break

def prune_branches(skeleton):
    """Removes unwanted edges from the skeleton, i.e. edges with the same
    face on both its sides
    """
    for he in skeleton.half_edges.itervalues():
        assert he.face.id is not None, he.id
        assert he.twin.face.id is not None, he.id
    # remove edges that have the same face on both sides
    remove = set()
    for he in skeleton.half_edges.itervalues():
        if he.face is he.twin.face:
            remove.add(he.id)
    for edge_id in remove:
        skeleton.remove_edge(edge_id, remove_nodes=True)

def define_groups(skeleton):
    """Defines groups of edges to be merged into longer chains
    """
    seen = set()
    groups = []
    for he in skeleton.half_edges.itervalues():
        if he in seen:
            continue
        seen.add(he)
        seen.add(he.twin)
        composed_of = deque([he])
        # we glue edges in both directions, using he.next and he.prev
        # walk in one direction
        start = he
        # glue edges, as long as:
        # - we have not yet seen the next edge we want to visit
        # - we do not jump over nodes with degree > 2
        # - we do not go from external edge to another external edge
        while he.next not in seen and \
            he.twin.origin.degree == 2 and \
            not (he.attrs['external'] and he.next.attrs['external']):
            he = he.next
            seen.add(he)
            seen.add(he.twin)
            composed_of.append(he)
        # walk in opposite direction
        he = start
        while he.prev not in seen and \
                he.origin.degree == 2 and \
                not (he.attrs['external'] and he.prev.attrs['external']):
            he = he.prev
            seen.add(he)
            seen.add(he.twin)
            composed_of.appendleft(he)
        groups.append(composed_of)
    return groups

def make_new_node_ids(skeleton, new_node_id):
    """Hands out new node identifiers for nodes that were not yet present """
    for node in skeleton.nodes.itervalues():
        # if this is not an integer, we have to hand out a new node id
        if not isinstance(node.id, int):
            new_node_id += 1
            node.id = new_node_id
    return new_node_id

def make_new_edges(groups, new_edge_id):
    """Constructs new edges based on the groups that were found by 
    the define_groups method
    """
    # once we have the groups, we can merge the chains into longer chains
    # this are the edges that have to be put back in the topology structure
    new_edges = []
    for new_edge_id, group in enumerate(groups, start = new_edge_id):
        geom = None
        for he in group:
            # take geometry
            if he.anchor is not None:
                # as is
                part = he.anchor.geometry[:]
            else:
                # reversed
                part = he.twin.anchor.geometry[::-1]
            # glue
            if geom is None:
                # this is the first part of the new geometry 
                geom = part
            else:
                # remove last point already there and add the new part to it
                geom.pop()
                geom.extend(part)
        new_edge = (new_edge_id, 
                    group[0].origin.id, group[-1].twin.origin.id, # node ids
                    group[0].face.id, group[0].twin.face.id,      # face ids
                    geom)
        new_edges.append(new_edge)
    # return both the new edges, as well as the highest edge identifier used
    return new_edges, new_edge_id