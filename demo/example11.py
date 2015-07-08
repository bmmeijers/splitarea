from simplegeom.wkt import loads
from tri import ToPointsAndSegments, triangulate
from predicates import orient2d
from tri.delaunay import InteriorTriangleIterator, ConvexHullTriangleIterator, TriangleIterator, output_triangles
from splitarea.flagging import EdgeEdgeHarvester, MidpointHarvester
from tri.delaunay import RegionatedTriangleIterator
from example12 import NarrowPartFinder
from splitarea.densify import densify
from pprint import pprint

with open('dh.wkt') as fh:
    lst = fh.readlines()
g = lst[1].split("\t")[0]

g = loads(g)
rings = []
for ring in g:
#     rings.append(densify(ring, small = 2.5))
    rings.append(ring)

conv = ToPointsAndSegments()
conv.add_polygon(rings)
dt = triangulate(conv.points, conv.infos, conv.segments)


# import sys
# sys.exit()
with open("/tmp/alltris.wkt", "w") as fh:
    output_triangles([t for t in TriangleIterator(dt)], fh)

# http://www.efunda.com/math/areas/CircleInscribeTriangleGen.cfm
# with side lengths a, b and c:
# radius = triangulararea / k = sqrt (k * (k-a) * (k-b) * (k-c)) / k, where k = 0.5 * (a + b + c)

# interior = [t for g,t in filter(lambda x: x[0] == 2, [(g,t) for (g,t) in RegionatedTriangleIterator(dt)])]
# two_tris = []
# for t in interior:
#     ct = 0
#     for i in xrange(3):
#         if t.constrained[i]:
#             ct += 1
#     if ct == 2:
#         two_tris.append(t)
# with open("/tmp/two_triangles.wkt", "w") as fh:
#     output_triangles(two_tris, fh)
for item in RegionatedTriangleIterator(dt):
    print item

interior = [t for g,d,t in filter(lambda x: x[0] == 1, [(g,d,t) for (g,d,t) in RegionatedTriangleIterator(dt)])]
visitor = NarrowPartFinder(interior, threshold = 80.)
visitor.find_narrows()
with open("/tmp/filtered.wkt", "w") as fh:
    output_triangles(visitor.filtered, fh)
with open("/tmp/remaining.wkt", "w") as fh:
    l = []
    filtered = frozenset(visitor.filtered)
    for t in ConvexHullTriangleIterator(dt):
        if t not in filtered:
            l.append(t) 
    output_triangles(l, fh)

with open("/tmp/path.wkt", "w") as fh:
    fh.write("i;group;depth;wkt\n")
    it = RegionatedTriangleIterator(dt)
    for i, (g, d, t) in enumerate(it, start = 1):
        fh.write("{0};{1};{2};{3}\n".format(i, g, d, t))

it = RegionatedTriangleIterator(dt)
zero_tris = []
for (g, d, t) in it:
    if g == 1:
        for i in xrange(3):
            if t.constrained[i]:
                break
        else:
            zero_tris.append(t)
with open("/tmp/outside_zero.wkt", "w") as fh:
    output_triangles(zero_tris, fh)

visitor = EdgeEdgeHarvester([t for t in ConvexHullTriangleIterator(dt)])
visitor.skeleton_segments()
with open("/tmp/skel0.wkt", "w") as fh:
    fh.write("wkt\n")
    for seg in visitor.segments:
        fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))