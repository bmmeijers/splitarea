from simplegeom.wkt import loads
from tri import ToPointsAndSegments, triangulate
from predicates import orient2d
from tri.delaunay import InteriorTriangleIterator, ConvexHullTriangleIterator, TriangleIterator, output_triangles
from tri.delaunay import RegionatedTriangleIterator
from splitarea.flagging import EdgeEdgeHarvester, MidpointHarvester
import json
from splitarea.densify import densify
from pprint import pprint



# http://www.efunda.com/math/areas/CircleInscribeTriangleGen.cfm
# with side lengths a, b and c:
# radius = triangulararea / k = sqrt (k * (k-a) * (k-b) * (k-c)) / k, where k = 0.5 * (a + b + c)

def height(a, b, c):
    base = a.distance(b)
    if base > 0:
        height = abs(orient2d(a, b, c)) / base
        return round(height * 2.) * .5

class NarrowPartFinder(object):
    def __init__(self, triangles, threshold=30.):
        self.triangles = triangles
        self.filtered = []
        self.threshold = threshold

    def process_0triangle(self, t):
        assert not t.constrained[0]
        assert not t.constrained[1]
        assert not t.constrained[2]
        a, b, c = t.vertices # t.origin, t.next.origin, t.next.next.origin
        L = [(a, b, c), (b, c, a), (c, a, b)]
        ct = 0
        for item in L:
            a, b, c, = item
            h =  height(a, b, c)
            if h is not None and h < self.threshold:
                ct += 1
        if ct > 2: # all 3 sides above threshold
            self.filtered.append(t)

    def process_1triangle(self, t, side):
        assert t.constrained[side]
        assert not t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
#         assert he.constraint
#         assert not he.next.constraint
#         assert not he.next.next.constraint
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3] # he.origin, he.next.origin, he.next.next.origin
        # Get length of the base of the triangle and its area, this gets the 
        # height of the triangle. 
        # This height is an approximation of the river width at this point
        # FIXME: I do not know whether I do calculate the length of the unconstrained edge here, yes or no
        h =  height(a, b, c)
        if h is not None and h < self.threshold:
            self.filtered.append(t)
#         base = a.distance(b)
#         if base > 0:
#             height = abs(orient2d(a, b, c)) / base
#             width = round(height * 2.) * .5
#             print base, height, width
#             if width < 10:
#                 self.filtered.append(t)
#         else:
#             # prevent div by zero
#             #height = 0.
#             pass

    def process_2triangle(self, t, side):
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert not t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        h =  height(a, b, c)
        if h is not None and h < self.threshold:
            self.filtered.append(t)

    def process_3triangle(self, t):
        side = 0
        assert t.constrained[side]
        assert t.constrained[(side+1)%3]
        assert t.constrained[(side+2)%3]
        a, b, c = t.vertices[(side+1)%3], t.vertices[(side+2)%3], t.vertices[(side+3)%3]
        

    def find_narrows(self):
        # create segments using knowledge on internal triangles
        tp = [1, 2, 4]
        for t in self.triangles:
            # type of triangle (no. of constraints)
            triangle_type = 0
            for i, c in enumerate(t.constrained):
                if c:
                    triangle_type += tp[i]
#             if t.constraint:
#                 triangle_type += 1
#             if t.next.constraint:
#                 triangle_type += 2
#             if t.next.next.constraint:
#                 triangle_type += 4
            # 0 - triangle
            if triangle_type == 0:
                print '0 triangle'
                self.process_0triangle(t)
            # 1 - triangle (3 rotations)
            elif triangle_type in (1, 2, 4):
                print "1 triangle"
                if triangle_type == 1:
                    self.process_1triangle(t, 0)
                elif triangle_type == 2:
                    self.process_1triangle(t, 1)
                elif triangle_type == 4:
                    self.process_1triangle(t, 2)
            # 2 - triangle (3 rotations)
            elif triangle_type in (3, 5, 6):
                print "2 triangle"
                if triangle_type == 3: # 1 + 2
                    self.process_2triangle(t, 0)
                elif triangle_type == 6: # 2 + 4
                    self.process_2triangle(t, 1)
                elif triangle_type == 5: # 4 + 1
                    self.process_2triangle(t, 2)
            # 3 - triangle
            elif triangle_type == 7:
                print "3 triangle"
                self.process_3triangle(t)

def main():
    with open('curvy.geojson') as fh:
        t = json.load(fh)
    
    rings = []
    for ring in t['features'][0]['geometry']['coordinates']:
        rings.append(
                     #densify(
                             [tuple(pt) for pt in ring[0]]
                     #        )
                     )
    
    conv = ToPointsAndSegments()
    conv.add_polygon(rings)
    dt = triangulate(conv.points, conv.infos, conv.segments)
    
    with open("/tmp/alltris.wkt", "w") as fh:
        output_triangles([t for t in TriangleIterator(dt)], fh)
    
    visitor = NarrowPartFinder([t for t in ConvexHullTriangleIterator(dt)])
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
        else:
            break
    with open("/tmp/outside_zero.wkt", "w") as fh:
        output_triangles(zero_tris, fh)

    visitor = EdgeEdgeHarvester([t for t in ConvexHullTriangleIterator(dt)])
    visitor.skeleton_segments()
    with open("/tmp/skel0.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

if __name__ == "__main__":
    main()