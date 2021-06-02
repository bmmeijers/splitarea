import unittest

from random import randint, seed
from collections import defaultdict, namedtuple

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads

from splitarea.harvest import EdgeEdgeHarvester, VertexInfo
from splitarea.weighted_harvest import EdgeEdgeWeightedHarvester, VertexInfo as VertexInfoWeights
from splitarea.harvest import ConnectorPicker
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
     (1005, 4, 1, 501, -1, LineString([(0,120), (0,110)])),
     (1004, 5, 2, -1, 501, LineString([(10,120), (10,110)]))
    ]
    # polygon edges
    polygon = [
        (1001, 1, 2, 501, 500, LineString([(0, 110), (5, 109), (10, 110)]) ),
        (1002, 1, 2, 500, -1, LineString([(0,110), (0,100), (10,100), (10,110)]) ),
    ]
    return external, polygon

class TestPolygonWithoutWeights(unittest.TestCase):

    def test_square(self):
        conv = ToPointsAndSegments()
        outside_edges, polygon = recs()
        for edge_id, start_node_id, end_node_id, _, _, geom in polygon:
            face = None
            weights = []
            for i, pt in enumerate(geom):
                if i == len(geom) - 1: # last pt
                    node = end_node_id
                    tp = 1
                    weights = [0, 10]
                elif i == 0: # first pt
                    node = start_node_id
                    tp = 1
                    weights = [0, 10]
                else: # intermediate pt
                    node = None
                    tp = 0
                    if edge_id == 1001:
                        weights = [10]
                    elif edge_id == 1002:
                        weights = [0]
                    else:
                        raise ValueError('encountered unknown edge')
                conv.add_point((pt.x, pt.y), VertexInfo(tp, face, node))
            for (start, end) in zip(geom[:-1],geom[1:]):
                (sx, sy) = start
                (ex, ey) = end
                conv.add_segment((sx, sy), (ex,ey))

        points, segments, infos = conv.points, conv.segments, conv.infos
        dt = triangulate(points, infos, segments)

#         for v in dt.vertices:
#             print v.info

        visitor = EdgeEdgeHarvester([t for t in InteriorTriangleIterator(dt)])
        visitor.skeleton_segments()

#         with open("/tmp/skel0_unweighted.wkt", "w") as fh:
#             fh.write("wkt\n")
#             for seg in visitor.segments:
#                 fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))
#     
#         with open("/tmp/skel1_unweighted.wkt", "w") as fh:
#             fh.write("wkt\n")
#             for lst in visitor.bridges.itervalues():
#                 for seg in lst:
#                     fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

        pick = ConnectorPicker(visitor)
        pick.pick_connectors()

        skeleton, new_edge_id = make_graph(outside_edges, visitor, new_edge_id=10000, universe_id=0, srid=-1)
        label_sides(skeleton)

#         lines = ["id;geometry\n"]
#         for he in skeleton.half_edges.itervalues():
#             lines.append("{};{}\n".format(he.id, he.geometry))
#         with open("/tmp/edges.wkt", "w") as fh:
#             fh.writelines(lines)

        # -- remove unwanted skeleton parts (same face id on both sides)
        prune_branches(skeleton)
        groups = define_groups(skeleton)
        new_edges, new_edge_id = make_new_edges(groups, new_edge_id)
        assert new_edge_id == 10000
        assert len(new_edges) == 1
        assert new_edges[0] == (10000, 5, 4, -1, 501, LineString([Point(x=10.0, y=120.0, srid=0), Point(x=10.0, y=110.0, srid=0), Point(x=7.5, y=104.5, srid=0), Point(x=2.5, y=104.5, srid=0), Point(x=0.0, y=110.0, srid=0), Point(x=0.0, y=120.0, srid=0)], srid=0))

class TestPolygonWithWeights(unittest.TestCase):
    def test_square(self):
        conv = ToPointsAndSegments()
        outside_edges, polygon = recs()
        for edge_id, start_node_id, end_node_id, _, _, geom in polygon:
            face = None
            weights = []
            for i, pt in enumerate(geom):
                if i == len(geom) - 1: # last pt
                    node = end_node_id
                    tp = 1
                    weights = [0, 10]
                elif i == 0: # first pt
                    node = start_node_id
                    tp = 1
                    weights = [0, 10]
                else: # intermediate pt
                    node = None
                    tp = 0
                    if edge_id == 1001:
                        weights = [10]
                    elif edge_id == 1002:
                        weights = [0]
                    else:
                        raise ValueError('encountered unknown edge')
                conv.add_point((pt.x, pt.y), VertexInfoWeights(tp, face, node, weights))
            for (start, end) in zip(geom[:-1],geom[1:]):
                (sx, sy) = start
                (ex, ey) = end
                conv.add_segment((sx, sy), (ex,ey))

        points, segments, infos = conv.points, conv.segments, conv.infos
        dt = triangulate(points, infos, segments)

#         for v in dt.vertices:
#             print v.info

        visitor = EdgeEdgeWeightedHarvester([t for t in InteriorTriangleIterator(dt)])
        visitor.skeleton_segments()

#         with open("/tmp/skel0.wkt", "w") as fh:
#             fh.write("wkt\n")
#             for seg in visitor.segments:
#                 fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))
#     
#         with open("/tmp/skel1.wkt", "w") as fh:
#             fh.write("wkt\n")
#             for lst in visitor.bridges.itervalues():
#                 for seg in lst:
#                     fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))


        pick = ConnectorPicker(visitor)
        pick.pick_connectors()

        skeleton, new_edge_id = make_graph(outside_edges, visitor, new_edge_id=10000, universe_id=0, srid=-1)
        label_sides(skeleton)

#         lines = ["id;geometry\n"]
#         for he in skeleton.half_edges.itervalues():
#             lines.append("{};{}\n".format(he.id, he.geometry))
#         with open("/tmp/edgesweights.wkt", "w") as fh:
#             fh.writelines(lines)

        # -- remove unwanted skeleton parts (same face id on both sides)
        prune_branches(skeleton)
        groups = define_groups(skeleton)
        new_edges, new_edge_id = make_new_edges(groups, new_edge_id)

        assert len(new_edges) == 1
        assert new_edges[0] == (10000, 5, 4, -1, 501, LineString([Point(x=10.0, y=120.0, srid=0), Point(x=10.0, y=110.0, srid=0), Point(x=10.0, y=100.0, srid=0), Point(x=0.0, y=100.0, srid=0), Point(x=0.0, y=110.0, srid=0), Point(x=0.0, y=120.0, srid=0)], srid=0))

if __name__ == '__main__':
#     ext, pol = recs()
#     for e in ext:
#         print e[5]
#     
#     for l in pol:
#         print l[5]
    unittest.main()