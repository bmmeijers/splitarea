from pprint import pprint

from connection import open_db

from tri.delaunay import ToPointsAndSegments, triangulate
from tri.delaunay import output_triangles, ConvexHullTriangleIterator

from splitarea.flagging import MidpointHarvester

conv = ToPointsAndSegments()
sql = "SELECT geom FROM landuse_vln"

with open_db(True) as db:
    for multi, in db.irecordset(sql):
        for poly in multi:
            conv.add_polygon(poly)

pprint(conv.points[:3])
print("...")
pprint(conv.points[-3:])

pprint(conv.segments[:3])
print("...")
pprint(conv.segments[-3:])

dt = triangulate(conv.points, conv.segments)
with open("/tmp/hull.wkt", "w") as fh:
    output_triangles([t for t in ConvexHullTriangleIterator(dt)], fh)

visitor = MidpointHarvester([t for t in ConvexHullTriangleIterator(dt)])
visitor.skeleton_segments()
with open("/tmp/skel.wkt", "w") as fh:
    fh.write("wkt\n")
    for seg in visitor.segments:
        fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))