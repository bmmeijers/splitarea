from math import hypot
from operator import sub

from tri.delaunay import orient2d

def dist(v0, v1):
    """ cartesian distance """
    dx = v0[0] - v1[0]
    dy = v0[1] - v1[1]
    return hypot(dx, dy)

def vector(end, start):
    """Creates a vector from the start to the end.

    It calculates based on two points: end -(minus) start.
    """
    return tuple(map(sub, end, start))

def normalize(v):
    """Normalizes the length of the vector with 2 elements
    """
    lengthv = hypot(v[0], v[1])
    assert lengthv != 0, "Vector without length cannot be normalized: {0}".format(v)
    return tuple([i/lengthv for i in v])

def vector_mul_scalar(v, s):
    """Multiply vector v with a scalar value s
    """
    return tuple([i * s for i in v])

def vector_add_vector(v0, v1):
    """Multiply vector v with a scalar value s
    """
    return tuple(map(sum, zip(v0, v1)))

TPL = "({0[0]}, {0[1]})"

# a sample polygon for which we add some Steiner points
# we would like to have more 2 triangles that end in a convex
# vertex this way, but this sample shows that we cannot do that
# up front completely
ring = [(0,0), (9.95,0), (12.3,1.2), (5, 6.3), (-1.1, 2)]
N = len(ring)
for i in xrange(N):
    prv, cur, nxt = ring[i-1], ring[i], ring[(i+1) % N]
    if orient2d(prv, cur, nxt) > 0:
        distp, distn = dist(prv, cur), dist(cur, nxt)
        frac = min(0.5 * distp, 0.5*distn)
        #if frac > 1.:
        #    frac = 1. / frac
        vecp = normalize(vector(prv, cur))
        vecn = normalize(vector(nxt, cur))
        
        toprv = vector_mul_scalar(vecp, frac)
        tonxt = vector_mul_scalar(vecn, frac)
        print TPL.format(cur)
        print TPL.format(vector_add_vector(cur, toprv))
        print TPL.format(vector_add_vector(cur, tonxt))
        print ""
        #print "POINT(", cur[0]+(cur[0]+nxt[0])* frac * .5, cur[1]+(cur[1]+nxt[1])* frac *.5, ")"


for pt in ring:
    print TPL.format(pt)
    
from tri import triangulate
from tri.delaunay import output_triangles
pts = [
    (0, 0),
(-0.55, 1.0),
(1.14127122105, 0.0),
(9.95, 0),
(8.6306725198, 0.0),
(11.125, 0.6),
(12.3, 1.2),
(11.125, 0.6),
(11.2184692423, 1.95558998144),
(5, 6.3),
(8.05903093145, 4.16286880131),
(1.95, 4.15),
(-1.1, 2),
(-0.167194488608, 2.65755142606),
(-0.55, 1.0)]
T = triangulate(list(set(pts)))
with open("/tmp/ptsonly.wkt", "w") as fh:
    output_triangles(T.triangles, fh)