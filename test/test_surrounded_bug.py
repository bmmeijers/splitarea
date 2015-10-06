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
    external = \
    [[1004953,
      2,
      1004476,
      1006293,
      1006100,
      LineString([Point(x=-201.894, y=-305.203, srid=0), Point(x=-202.667, y=-311.323, srid=0), Point(x=-193.394, y=-315.9925, srid=0), Point(x=-193.564, y=-316.7255, srid=0)], srid=0)],
     [1010199,
      1004476,
      1007143,
      1006293,
      1006100,
      LineString([Point(x=-193.564, y=-316.7255, srid=0), Point(x=-203.348, y=-314.259, srid=0), Point(x=-203.594, y=-337.973, srid=0), Point(x=-202.365, y=-347.802, srid=0), Point(x=-194.872, y=-383.102, srid=0), Point(x=-187.366, y=-420.966, srid=0), Point(x=-183.5145, y=-423.9925, srid=0)], srid=0)],
     [1010646,
      1331,
      1333,
      1004425,
      1006293,
      LineString([Point(x=-13.523, y=-314.854, srid=0), Point(x=-11.0355, y=-313.1515, srid=0), Point(x=-11.158, y=-311.596, srid=0), Point(x=-14.004, y=-308.74, srid=0), Point(x=-20.565, y=-309.257, srid=0)], srid=0)]]
    # edges defining the polygon to be split
    polygon = \
    [[1903,
      1333,
      1332,
      1004425,
      1006033,
      LineString([Point(x=-20.565, y=-309.257, srid=0), Point(x=-20.292, y=-312.717, srid=0), Point(x=-20.084, y=-315.361, srid=0)], srid=0)],
     [1005389,
      1332,
      1331,
      1004425,
      1006033,
      LineString([Point(x=-20.084, y=-315.361, srid=0), Point(x=-17.239, y=-315.141, srid=0), Point(x=-13.523, y=-314.854, srid=0)], srid=0)],
     [1009934,
      1004476,
      1333,
      1006293,
      1006033,
      LineString([Point(x=-193.564, y=-316.7255, srid=0), Point(x=-183.78, y=-319.192, srid=0), Point(x=-181.459, y=-319.701, srid=0), Point(x=-176.668, y=-319.862, srid=0), Point(x=-156.915, y=-320.53, srid=0), Point(x=-96.624, y=-314.452, srid=0), Point(x=-51.233, y=-311.671, srid=0), Point(x=-20.565, y=-309.257, srid=0)], srid=0)],
     [1010437,
      1331,
      1004476,
      1006293,
      1006033,
      LineString([Point(x=-13.523, y=-314.854, srid=0), Point(x=-14.227, y=-324.443, srid=0), Point(x=-16.649, y=-328.969, srid=0), Point(x=-22.207, y=-341.633, srid=0), Point(x=-30.853, y=-351.518, srid=0), Point(x=-40.735, y=-356.769, srid=0), Point(x=-55.247, y=-359.549, srid=0), Point(x=-94.772, y=-361.402, srid=0), Point(x=-119.344, y=-362.725, srid=0), Point(x=-154.431, y=-362.655, srid=0), Point(x=-169.850800842, y=-363.79366208, srid=0), Point(x=-173.407, y=-363.87, srid=0), Point(x=-175.696, y=-363.142, srid=0), Point(x=-183.532, y=-358.662, srid=0), Point(x=-186.713, y=-353.063, srid=0), Point(x=-190.159, y=-346.818, srid=0), Point(x=-192.098, y=-338.851, srid=0), Point(x=-191.236, y=-332.391, srid=0), Point(x=-184.571, y=-322.8, srid=0), Point(x=-193.7425, y=-317.5375, srid=0), Point(x=-193.564, y=-316.7255, srid=0)], srid=0)],
     [1010852,
      1332,
      1332,
      1006033,
      1006189,
      LineString([Point(x=-20.084, y=-315.361, srid=0), Point(x=-19.737, y=-321.247, srid=0), Point(x=-24.369, y=-332.676, srid=0), Point(x=-32.089, y=-344.105, srid=0), Point(x=-37.955, y=-349.665, srid=0), Point(x=-45.366, y=-351.827, srid=0), Point(x=-56.483, y=-353.062, srid=0), Point(x=-82.112, y=-353.371, srid=0), Point(x=-117.623, y=-356.46, srid=0), Point(x=-165.611, y=-359.953, srid=0), Point(x=-173.148, y=-358.231, srid=0), Point(x=-181.115, y=-353.494, srid=0), Point(x=-185.853, y=-346.818, srid=0), Point(x=-186.929, y=-340.143, srid=0), Point(x=-184.561, y=-333.683, srid=0), Point(x=-176.811, y=-324.08, srid=0), Point(x=-156.583, y=-325.007, srid=0), Point(x=-127.194, y=-324.027, srid=0), Point(x=-76.245, y=-319.702, srid=0), Point(x=-20.084, y=-315.361, srid=0)], srid=0)]]
    return external, polygon

class TestPolygonWithHole(unittest.TestCase):

    def test_polygon_with_hole(self):
        """Splitting of face with for which the skeleton is a cycle 
        There should be a bridge connector made at node 1332
        """
        conv = ToPointsAndSegments()
        outside_edges, polygon = recs()
        for edge_id, start_node_id, end_node_id, _, _, geom in polygon:
            face = None
            # loop edge: 1010852 (start node == end node)
            for i, pt in enumerate(geom):
                if i == len(geom) - 1: # last pt
                    node = end_node_id
                    tp = 1
                    if node == 1332: # we need a bridge vertex
                        face = 1006189
                        tp = 2
                        print "setting type to", tp, "face to", face
                elif i == 0: # first pt
                    node = start_node_id
                    tp = 1
                    if node == 1332:
                        face = 1006189
                        tp = 2
                        print "setting type to", tp, "face to", face
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

        pick = ConnectorPicker(visitor)
        pick.pick_connectors()

        with open("/tmp/skel0.wkt", "w") as fh:
            fh.write("wkt\n")
            for seg in visitor.segments:
                fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))
        with open("/tmp/skel1.wkt", "w") as fh:
            fh.write("wkt\n")
            for lst in visitor.bridges.itervalues():
                for seg in lst:
                    fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))


        skeleton, new_edge_id = make_graph(outside_edges, visitor, new_edge_id=10000, universe_id=0, srid=-1)
        label_sides(skeleton)
#         assert new_edge_id == 10001

        # -- remove unwanted skeleton parts (same face id on both sides)
        prune_branches(skeleton)
        groups = define_groups(skeleton)
        new_edges, new_edge_id = make_new_edges(groups, new_edge_id)
        from pprint import pprint
        pprint(new_edges)
        with open("/tmp/edges_new.wkt", "w") as fh:
            fh.write("eid;sn;en;lf;rf;geom\n")
            for eid, sn, en, lf, rf, geom, in new_edges:
                print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", geom

#         assert len(new_edges) == 1
#         assert new_edge_id == 10001

if __name__ == '__main__':
    unittest.main()
