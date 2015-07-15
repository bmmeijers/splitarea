from random import randint, seed
from collections import defaultdict, namedtuple

# from mesher.mesh import Mesh, Vertex
# from mesher.utils import MeshVisualizer

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads
    
from splitarea.harvest import EdgeEdgeHarvester, VertexInfo, ConnectorPicker
from splitarea.skeleton import SkeletonGraph

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
    ]
    # polygon edges
    polygon = [
        (331, 
         199, 199,
         74, 67,
         LineString([Point(x=561248.71376, y=5898194.94024, srid=32632),
                     Point(x=561266.86318, y=5898180.95802, srid=32632),
                     Point(x=561265.84957, y=5898172.80328, srid=32632),
                     Point(x=561264.9987, y=5898171.09707, srid=32632),
                     Point(x=561262.3693, y=5898140.07594, srid=32632),
                     Point(x=561227.39909, y=5898123.38065, srid=32632),
                     Point(x=561210.85369, y=5898130.78853, srid=32632),
                     Point(x=561180.97198, y=5898161.24993, srid=32632),
                     Point(x=561176.4612, y=5898169.25525, srid=32632),
                     Point(x=561172.95066, y=5898180.10562, srid=32632),
                     Point(x=561173.61721, y=5898180.74667, srid=32632),
                     Point(x=561187.34796, y=5898187.46103, srid=32632),
                     Point(x=561201.76614, y=5898190.8415, srid=32632),
                     Point(x=561243.65464, y=5898195.51012, srid=32632),
                     Point(x=561248.71376, y=5898194.94024, srid=32632)], srid=32632))
    ]
    return external, polygon

def test():

#    Test:
#    =====
#    - Hole that is dissolved into parent (actually this is a merge!)

    conv = ToPointsAndSegments()
    
    outside_edges, polygon = recs()
    for _, start_node_id, end_node_id, _, _, geom in polygon: 
        for i, pt in enumerate(geom):
            if i == len(geom) - 1: # last pt
                node = end_node_id
                tp = 2
            elif i == 0: # first pt
                node = start_node_id
                tp = 2
            else: # intermediate pt
                node = None
                tp = 0
            conv.add_point((pt.x, pt.y), VertexInfo(tp, 67, node))

        for (start, end) in zip(geom[:-1],geom[1:]):
            (sx, sy) = start
            (ex, ey) = end
            conv.add_segment((sx, sy), (ex,ey))

    points, segments, infos = conv.points, conv.segments, conv.infos

    pprint(points)
    pprint(segments)
    pprint(infos)

    dt = triangulate(points, infos, segments)

    for vertex in dt.vertices:
        print "POINT(", vertex.x, vertex.y, ")" #vertex.info

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
            print lst
            for seg in lst:
                fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    pick = ConnectorPicker(visitor)
    pick.pick_connectors()

    with open("/tmp/skel2.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.ext_segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

#     outside_edges = [
# #         #eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf
#         (4000,
#          LineString([d, b],),
#          5001, False, 5001,
#          1001, False, 1001,
#          "A", "B"
#         ),
#         (4001,
#          LineString([e, c],),
#          5002, False, 5002,
#          1002, False, 1002,
#          "B", "A"
#          ),
#     ] 
#     outside_edges = []
    skeleton = SkeletonGraph()
    skeleton.srid = 32632
    print """
    
    ADDING OUTSIDE EDGES
    """
    # first add outside edges
    for outside_edge in outside_edges:
        eid, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf, geom, = outside_edge
        skeleton.add_segment(geom, 
                             external = True, 
                             edge_id = eid, 
                             left_face_id = lf, 
                             right_face_id = rf,
                             start_node_id = sn, 
                             end_node_id = en,
                             start_vertex_id = svtxid,
                             end_vertex_id = evtxid,
                             start_external = sn_ext,
                             end_external = en_ext
                             )
    print """
      
    BRIDGE CONNECTORS
    """
#     # add nodes from inner rings ("bridge" connectors)
    for i, segment in enumerate(visitor.ext_segments):
        
        v0, v1, lf, rf = segment
        ln = LineString(srid=skeleton.srid)
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        print segment
        print ln
        skeleton.add_segment(ln, 
                             external = True, 
                             edge_id = i,
                             start_vertex_id = v0.info.vertex_id,
                             end_vertex_id = v1.info.vertex_id,
                             left_face_id = lf,
                             right_face_id = rf,
#                             start_external = True,
#                             end_external = True
                             )
 
    print """
     
    UNLABELED EDGES
    """
#     # then add all segments which are unlabeled
    for i, segment in enumerate(visitor.segments, start = len(visitor.ext_segments)):
        v0, v1, = segment
        ln = LineString(srid=skeleton.srid)
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        start_vertex_id = v0.info.vertex_id
        if start_vertex_id is None:
            start_vertex_id = (id(v0), ) # note, not an int but tuple to prevent duplicate with external ids
        end_vertex_id = v1.info.vertex_id
        if end_vertex_id is None:
            end_vertex_id = (id(v1), ) # note, not an int but tuple to prevent duplicate with external ids
        print start_vertex_id
        print end_vertex_id
        skeleton.add_segment(ln, 
                             start_vertex_id = start_vertex_id,
                             end_vertex_id = end_vertex_id,
                             external = False, 
                             edge_id = i)

    with open('/tmp/edges.wkt', 'w') as fh:
        fh.write("geometry;leftface;rightface;sn;en\n")
        for edge in skeleton.edges:
            print >> fh, edge.geometry,";", edge.left_face_id,";", edge.right_face_id, ";", edge.start_node.id,";", edge.end_node.id

    skeleton.label_sides()
    skeleton.prune_branches()
    skeleton.find_new_edges(new_edge_id=90000, new_node_id=80000)
    skeleton.visualize_nodes()
    with open("/tmp/edges_new.wkt", "w") as fh:
        fh.write("eid;sn;en;lf;rf;length;geom\n")
        for eid, sn, en, lf, rf, length, geom, in skeleton.new_edges:
            print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", length,";",  geom

if __name__ == '__main__':
    test()
