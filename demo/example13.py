from random import randint, seed
from collections import defaultdict, namedtuple

# from mesher.mesh import Mesh, Vertex
# from mesher.utils import MeshVisualizer

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads
    
from splitarea.harvest import EdgeEdgeHarvester, VertexInfo, ConnectorPicker
from splitarea.skeleton import make_graph, label_sides,\
    prune_branches, define_groups, make_new_edges

from tri import triangulate, ToPointsAndSegments # polygon_as_points_and_segments
from tri.delaunay import output_triangles, output_vertices
from tri.delaunay import TriangleIterator, InteriorTriangleIterator, ConvexHullTriangleIterator

from pprint import pprint
import sys

def angle(pa, pb):
    from math import atan2
    dx = pb[0] - pa[0]
    dy = pb[1] - pa[1]
    return atan2(dy, dx)

def recs():
    # external edge
    # edge_id, start_node, end_node, left_face, right_face, geom
    external = [
    # The true/false for a node represents whether it touches the boundary of the polygon
    # if i recall correctly...
    # eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf,
     (252, 135, 130, 54, 45, LineString([Point(x=560850.10666, y=5897605.99348, srid=32632), Point(x=560741.10142, y=5897548.82285, srid=32632), Point(x=560721.14836, y=5897552.58643, srid=32632)], srid=32632))
    ,(253, 130, 141, 74, 45, LineString([Point(x=560721.14836, y=5897552.58643, srid=32632), Point(x=560722.71918, y=5897569.99831, srid=32632), Point(x=560693.83728, y=5897603.2688, srid=32632), Point(x=560756.86961, y=5897619.73413, srid=32632), Point(x=560777.04418, y=5897623.14318, srid=32632), Point(x=560797.7284, y=5897626.93212, srid=32632), Point(x=560848.93159, y=5897639.2497, srid=32632)], srid=32632))
    ,(275, 146, 149, 74, 51, LineString([Point(x=560852.35323, y=5897649.45567, srid=32632), Point(x=560817.76284, y=5897681.17662, srid=32632), Point(x=560795.42893, y=5897701.01108, srid=32632), Point(x=560832.65749, y=5897740.48761, srid=32632), Point(x=560836.92066, y=5897740.48056, srid=32632), Point(x=560861.8534, y=5897713.28732, srid=32632), Point(x=560861.9305, y=5897709.23867, srid=32632), Point(x=560894.51146, y=5897674.45864, srid=32632)], srid=32632))
    ,(290, 142, 166, 74, 54, LineString([Point(x=560923.98887, y=5897644.8424, srid=32632), Point(x=560924.54319, y=5897645.13689, srid=32632), Point(x=560951.61553, y=5897659.51281, srid=32632), Point(x=561160.70759, y=5897771.29099, srid=32632), Point(x=561206.42491, y=5897795.92407, srid=32632), Point(x=561218.48727, y=5897804.06678, srid=32632)], srid=32632))
    ]
    # polygon edges
    polygon = [
    (251, 141, 135, 48, 45, LineString([Point(x=560848.93159, y=5897639.2497, srid=32632), Point(x=560863.31688, y=5897626.61676, srid=32632), Point(x=560850.10666, y=5897605.99348, srid=32632)], srid=32632))
    ,(263, 142, 135, 54, 48, LineString([Point(x=560923.98887, y=5897644.8424, srid=32632), Point(x=560923.98066, y=5897644.78827, srid=32632), Point(x=560909.5673, y=5897637.17918, srid=32632), Point(x=560850.10666, y=5897605.99348, srid=32632)], srid=32632))
    ,(264, 141, 146, 74, 48, LineString([Point(x=560848.93159, y=5897639.2497, srid=32632), Point(x=560853.92697, y=5897645.61066, srid=32632), Point(x=560852.35323, y=5897649.45567, srid=32632)], srid=32632))
    ,(265, 146, 149, 51, 48, LineString([Point(x=560852.35323, y=5897649.45567, srid=32632), Point(x=560876.41272, y=5897659.30315, srid=32632), Point(x=560877.61799, y=5897658.14633, srid=32632), Point(x=560894.51146, y=5897674.45864, srid=32632)], srid=32632))
    ,(266, 149, 142, 74, 48, LineString([Point(x=560894.51146, y=5897674.45864, srid=32632), Point(x=560923.98887, y=5897644.8424, srid=32632)], srid=32632))
    ]
    return external, polygon

def test():

#    Test:
#    =====
#    Triangle with *touching* hole

#    FIXME: 
#    unnecessary nodes -->>> too short edges

    conv = ToPointsAndSegments()
    outside_edges, polygon = recs()
    for _, start_node_id, end_node_id, _, _, geom in polygon: 
        for i, pt in enumerate(geom):
            if i == len(geom) - 1: # last pt
                node = end_node_id
                tp = 1
            elif i == 0: # first pt
                node = start_node_id
                tp = 1
            else: # intermediate pt
                node = None
                tp = 0
            conv.add_point((pt.x, pt.y), VertexInfo(tp, None, node))

        for (start, end) in zip(geom[:-1],geom[1:]):
            (sx, sy) = start
            (ex, ey) = end
            conv.add_segment((sx, sy), (ex,ey))

    points, segments, infos = conv.points, conv.segments, conv.infos
    dt = triangulate(points, infos, segments)

    with open("/tmp/alltris.wkt", "w") as fh:
        output_triangles([t for t in TriangleIterator(dt)], fh)
    with open("/tmp/allvertices.wkt", "w") as fh:
        output_vertices(dt.vertices, fh)
    with open("/tmp/interiortris.wkt", "w") as fh:
        output_triangles([t for t in InteriorTriangleIterator(dt)], fh)
    with open("/tmp/hull.wkt", "w") as fh:
        output_triangles([t for t in ConvexHullTriangleIterator(dt)], fh)

    visitor = EdgeEdgeHarvester([t for t in InteriorTriangleIterator(dt)])
    visitor.skeleton_segments()
    with open("/tmp/skel0.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    with open("/tmp/skel1.wkt", "w") as fh:
        fh.write("wkt\n")
        for lst in visitor.bridges.itervalues():
            for seg in lst:
                fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    pick = ConnectorPicker(visitor)
    pick.pick_connectors()

    with open("/tmp/skel2.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.ext_segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    skeleton = make_graph(outside_edges, visitor)
    label_sides(skeleton)

    prune_branches(skeleton)
    groups = define_groups(skeleton)
    new_edges = make_new_edges(groups)
    with open("/tmp/edges_new.wkt", "w") as fh:
        fh.write("eid;sn;en;lf;rf;geom\n")
        for eid, sn, en, lf, rf, geom, in new_edges:
            print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", geom

# #     outside_edges = [
# # #         #eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf
# #         (4000,
# #          LineString([d, b],),
# #          5001, False, 5001,
# #          1001, False, 1001,
# #          "A", "B"
# #         ),
# #         (4001,
# #          LineString([e, c],),
# #          5002, False, 5002,
# #          1002, False, 1002,
# #          "B", "A"
# #          ),
# #     ] 
# #     outside_edges = []
#     skeleton = SkeletonGraph()
#     skeleton.srid = 32632
#     print """
#     
#     ADDING OUTSIDE EDGES
#     """
#     # first add outside edges
#     for outside_edge in outside_edges:
#         eid, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf, geom, = outside_edge
#         skeleton.add_segment(geom, 
#                              external = True, 
#                              edge_id = eid, 
#                              left_face_id = lf, 
#                              right_face_id = rf,
#                              start_node_id = sn, 
#                              end_node_id = en,
#                              start_vertex_id = svtxid,
#                              end_vertex_id = evtxid,
#                              start_external = sn_ext,
#                              end_external = en_ext
#                              )
#     print """
#      
#     BRIDGE CONNECTORS
#     """
# #     # add nodes from inner rings ("bridge" connectors)
#     for i, segment in enumerate(visitor.ext_segments):
#         print segment
#         v0, v1, lf, rf = segment
#         ln = LineString(srid=skeleton.srid)
#         ln.append(Point(v0.x, v0.y))
#         ln.append(Point(v1.x, v1.y))
#         skeleton.add_segment(ln, 
#                              external = True, 
#                              edge_id = i,
#                              start_vertex_id = v0.info.vertex_id,
#                              end_vertex_id = v1.info.vertex_id,
#                              left_face_id = lf,
#                              right_face_id = rf,
# #                             start_external = True,
# #                             end_external = True
#                              )
#  
#     print """
#      
#     UNLABELED EDGES
#     """
# #     # then add all segments which are unlabeled
#     for i, segment in enumerate(visitor.segments, start = len(visitor.ext_segments)):
#         v0, v1, = segment
#         ln = LineString(srid=skeleton.srid)
#         ln.append(Point(v0.x, v0.y))
#         ln.append(Point(v1.x, v1.y))
#         start_vertex_id = v0.info.vertex_id
#         if start_vertex_id is None:
#             start_vertex_id = (id(v0), ) # note, not an int but tuple to prevent duplicate with external ids
#         end_vertex_id = v1.info.vertex_id
#         if end_vertex_id is None:
#             end_vertex_id = (id(v1), ) # note, not an int but tuple to prevent duplicate with external ids
#         skeleton.add_segment(ln, 
#                              start_vertex_id = start_vertex_id,
#                              end_vertex_id = end_vertex_id,
#                              external = False, 
#                              edge_id = i)
# 
# 
#     skeleton.label_sides()
#     skeleton.prune_branches()
#     with open('/tmp/edges.wkt', 'w') as fh:
#         fh.write("geometry;id;leftface;rightface;sn;en\n")
#         for edge in skeleton.edges:
#             print >> fh, edge.geometry, ";", edge.edge_id, ";", edge.left_face_id,";", edge.right_face_id, ";", edge.start_node.id,";", edge.end_node.id
# 
#     skeleton.find_new_edges(new_edge_id=90000, new_node_id=80000)
#     skeleton.visualize_nodes()
#     with open("/tmp/edges_new.wkt", "w") as fh:
#         fh.write("eid;sn;en;lf;rf;length;geom\n")
#         for eid, sn, en, lf, rf, length, geom, in skeleton.new_edges:
#             print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", length,";",  geom

if __name__ == '__main__':
    test()
