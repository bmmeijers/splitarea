import unittest

from random import randint, seed
from collections import defaultdict, namedtuple

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
     (1005, 4, 1, 501, -1, LineString([(0,120), (0,110)])),
     (1004, 5, 2, -1, 501, LineString([(10,120), (10,110)]))
    ]
    # polygon edges
    polygon = [
        (1001, 1, 2, 501, 500, LineString([(0, 110), (10, 110)]) ),
        (1002, 1, 2, 500, -1, LineString([(0,110), (0,100), (10,100), (10,110)]) ),
        (1003, 3, 3, 500, -1, LineString([(5,102), (5,104), (7,104), (7,102), (5,102)]) )
    ]
    return external, polygon

class TestPolygonWithHole(unittest.TestCase):

    def test_polygon_with_hole(self):
        conv = ToPointsAndSegments()
        outside_edges, polygon = recs()
        for edge_id, start_node_id, end_node_id, _, _, geom in polygon:
            face = None
            if edge_id == 1003: # interior ring
                face = -1
            for i, pt in enumerate(geom):
                if i == len(geom) - 1: # last pt
                    node = end_node_id
                    tp = 1
                    if edge_id == 1003: # we need a bridge vertex
                        tp = 2
                elif i == 0: # first pt
                    node = start_node_id
                    tp = 1
                    if edge_id == 1003:
                        tp = 2
                else: # intermediate pt
                    node = None
                    tp = 0
                conv.add_point((pt.x, pt.y), VertexInfo(tp, face, node))
            for (start, end) in zip(geom[:-1],geom[1:]):
                (sx, sy) = start
                (ex, ey) = end
                conv.add_segment((sx, sy), (ex,ey))

        points, segments, infos = conv.points, conv.segments, conv.infos
        dt = triangulate(points, infos, segments)

        visitor = EdgeEdgeHarvester([t for t in InteriorTriangleIterator(dt)])
        visitor.skeleton_segments()

        pick = ConnectorPicker(visitor)
        pick.pick_connectors()

        skeleton, new_edge_id = make_graph(outside_edges, visitor, new_edge_id=10000, universe_id=0, srid=-1)
        label_sides(skeleton)
        assert new_edge_id == 10001

        # -- remove unwanted skeleton parts (same face id on both sides)
        prune_branches(skeleton)
        groups = define_groups(skeleton)
        new_edges, new_edge_id = make_new_edges(groups, new_edge_id)

        assert len(new_edges) == 1
        assert new_edge_id == 10001

if __name__ == '__main__':
    unittest.main()