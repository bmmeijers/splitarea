# from pprint import pprint

from simplegeom.geometry import LineString

from splitarea.harvest import EdgeEdgeHarvester, ConnectorPicker, VertexInfo

from splitarea.skeleton import make_graph, label_sides, prune_branches, define_groups, make_new_edges

from tri import triangulate, ToPointsAndSegments # polygon_as_points_and_segments
from tri.delaunay import output_triangles, output_vertices
from tri.delaunay import TriangleIterator, InteriorTriangleIterator, ConvexHullTriangleIterator



def test():

    conv = ToPointsAndSegments()
    conv.add_point((0,0), info = VertexInfo(0, None, None))
    conv.add_point((9,1), info = VertexInfo(1, None, 1001))
    conv.add_point((10,10), info = VertexInfo(0, None, None))
    conv.add_point((1,9), info = VertexInfo(1, None, 1002))
    conv.add_point((0,0), info = VertexInfo(0, None, None))

    conv.add_segment((0,0), (9,1))
    conv.add_segment((9,1), (10,10))
    conv.add_segment((10,10), (1,9))
    conv.add_segment((1,9), (0,0))

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

    pick = ConnectorPicker(visitor)
    pick.pick_connectors()

#     visitor = MidpointHarvester([t for t in InteriorTriangleIterator(dt)])
#     visitor.skeleton_segments()
#     with open("/tmp/skel.wkt", "w") as fh:
#         fh.write("wkt\n")
#         for seg in visitor.segments:
#             fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

#     with open("/tmp/centres.wkt", "w") as fh:
#         fh.write("wkt\n")
#         for t, point in visitor.triangle_point.iteritems():
#             fh.write("POINT({0})\n".format(point))

    outside_edges = [
        #eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf
        (4000,
         LineString([(0,20), (1,9)]),
         5002, True, (0,20),
         1002, False, (1,9),

         10000,
         10005
         ),
        (4001,
         LineString([(20,1), (9,1)]),

         5001, True, (20,1),
         1001, False, (9,1),

         10005,
         10000
         ),
    ] 

    skeleton = make_graph(outside_edges, visitor)
    label_sides(skeleton)
    prune_branches(skeleton)
    groups = define_groups(skeleton)
    new_edges = make_new_edges(groups)

    with open("/tmp/edges_new.wkt", "w") as fh:
        fh.write("eid;sn;en;lf;rf;geom\n")
        for eid, sn, en, lf, rf, geom, in new_edges:
            print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", geom

if __name__ == '__main__':
    test()
