import json

from tri.delaunay import ToPointsAndSegments, triangulate
from tri.delaunay import output_triangles, ConvexHullTriangleIterator, InteriorTriangleIterator

from splitarea.flagging import MidpointHarvester
from splitarea.densify import densify

with open('/home/martijn/workspace/splitarea/data/sandro/poly.geojson') as fh:
    c = json.loads(fh.read())

conv = ToPointsAndSegments()
poly = c['features'][0]['geometry']['coordinates']

rings = []
for ring in poly:
    rings.append( densify([tuple(pt) for pt in ring], 1))
del poly
conv.add_polygon(rings)

dt = triangulate(conv.points, None, conv.segments)
with open("/tmp/hull.wkt", "w") as fh:
    output_triangles([t for t in ConvexHullTriangleIterator(dt)], fh)

visitor = MidpointHarvester([t for t in InteriorTriangleIterator(dt)])
visitor.skeleton_segments()
with open("/tmp/skel.wkt", "w") as fh:
    fh.write("wkt\n")
    for seg in visitor.segments:
        fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))