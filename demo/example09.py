from random import randint, seed
from collections import defaultdict, namedtuple

# from mesher.mesh import Mesh, Vertex
# from mesher.utils import MeshVisualizer

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads
    
from splitarea.flagging import EdgeEdgeHarvester, VertexInfo
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

def test():

#    Test:
#    =====
#    Triangle with *touching* hole

    conv = ToPointsAndSegments()
    a, b, c, = (0,0), (10,20), (20,0)
    d, e = (10,25), (25,0) 
    g, h = (10,16), (16,2)

    alpha3 = angle(a, b)
    alpha2 = angle(a, g)
    alpha1 = angle(a, h)
    alpha0 = angle(a, c)
    
    # node sector
    # (he.twin.face.id, he.twin.next.id, he.twin.next.angle, he.id, he.angle)
    sectors = [
        ("Z", 9001, alpha0, 9002, alpha1),
        ("C", 9002, alpha1, 9002, alpha2),
        ("Z", 9002, alpha2, 9003, alpha3),
        ("B", 9003, alpha3, 9001, alpha0)
#         ("Z", alpha1, 9002, alpha0, 9001),
#         ("C", alpha2, 9002, alpha1, 9002),
#         ("Z", alpha3, 9003, alpha2, 9002),
#         ("B", alpha0, 9001, alpha3, 9003)
    ]
    conv.add_point(b, info = VertexInfo(1, None, 1001))
    conv.add_point(c, info = VertexInfo(0, None, 1002))

    conv.add_point(a, info = VertexInfo(3, sectors, 1003))

    conv.add_point(g, info = VertexInfo(0, None, None))
    conv.add_point(h, info = VertexInfo(0, None, None))

    conv.add_segment(a, b)
    conv.add_segment(b, c)
    conv.add_segment(c, a)

    conv.add_segment(a, g)
    conv.add_segment(g, h)
    conv.add_segment(h, a)

    points, segments, infos = conv.points, conv.segments, conv.infos

    pprint(points)
    pprint(segments)
    pprint(infos)

    dt = triangulate(points, infos, segments)

    for vertex in dt.vertices:
        print vertex, vertex.info

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

    visitor.pick_connectors()

    with open("/tmp/skel2.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.ext_segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    outside_edges = [
#         #eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf
        (4000,
         LineString([d, b],),
         5001, False, 5001,
         1001, False, 1001,
         "A", "B"
        ),
        (4001,
         LineString([e, c],),
         5002, False, 5002,
         1002, False, 1002,
         "B", "A"
         ),
    ] 
#     outside_edges = []
    skeleton = SkeletonGraph()
    print """
    
    ADDING OUTSIDE EDGES
    """
    # first add outside edges
    for outside_edge in outside_edges:
        eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf, = outside_edge
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
        print segment
        v0, v1, lf, rf = segment
        ln = LineString()
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
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
    for i, segment in enumerate(visitor.segments):
        v0, v1, = segment       
        ln = LineString()
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
                             edge_id = i + len(visitor.ext_segments))

    with open('/tmp/edges.wkt', 'w') as fh:
        fh.write("geometry;leftface;rightface;sn;en\n")
        for edge in skeleton.edges:
            print >> fh, edge.geometry,";", edge.left_face_id,";", edge.right_face_id, ";", edge.start_node.id,";", edge.end_node.id

    skeleton.visualize_nodes()
    skeleton.label_sides()
    skeleton.prune_branches()
    skeleton.find_new_edges(new_edge_id=90000, new_node_id=80000)

    with open("/tmp/edges_new.wkt", "w") as fh:
        fh.write("eid;sn;en;lf;rf;length;geom\n")
        for eid, sn, en, lf, rf, length, geom, in skeleton.new_edges:
            print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", length,";",  geom

if __name__ == '__main__':
    test()
