from simplegeom.geometry import Point, LineString
from math import hypot, floor, ceil
#from collections import deque

def dist(pa, pb):
    dx = pb[0] - pa[0]
    dy = pb[1] - pa[1]
    return hypot(dx,dy)
#
#def center(pa, pb):
#    return Point((pa[0]+pb[0])/2., (pa[1]+pb[1])/2.)
#
#pt0 = Point(0,0)
#pt1 = Point(2.5,0)
#pt2 = Point(30,50)
#pt3 = Point(100,50)
def divide(pt0, pt1, new, eps, inclusive_last = False):
    no = floor(dist(pt0, pt1) / eps)
    no = max(no, 1)
    dx = (pt1[0] - pt0[0]) / no
    dy = (pt1[1] - pt0[1]) / no
    total = int(no)
    if inclusive_last:
        total += 1
    for i in range(total):
        new.append(Point(pt0[0] + i*dx, pt0[1] + i*dy))
#
#ln = LineString()
#ln.append(pt0)
#ln.append(pt1)
#ln.append(pt2)
#ln.append(pt3)
#

def minimal_segment_length(ln):
    assert len(ln) > 1
    for j in range(1, len(ln)):
        pt0, pt1 = ln[j - 1], ln[j]
        min_dist = dist(pt0, pt1)
        if j == 1:
            smallest = min_dist
        else:
            if min_dist < smallest:
                smallest = min_dist
    return smallest

#print ln

def densify(ln, small = 10):

    new = LineString()
    stop = len(ln)
    last = stop - 1
    for j in range(1, stop):
        pt0, pt1 = ln[j - 1], ln[j]
        inclusive = False
        if j == last:
            inclusive = True
        divide(pt0, pt1, new, small, inclusive)
    # copy first and last back into line 
    # (to not break any dependencies in topology)
    new[0] = Point(*ln[0])
    new[len(new)-1] = Point(*ln[len(ln)-1])
    return new
#
#print densify(ln)
#
#            
#        
#
#stack = [(pt0, pt1)]
#L = [pt0]
#guard = 0
#while stack:
#    if guard > 50: break
#    guard += 1
#    pt0, pt1 = stack.pop()
#
#    d = dist(pt0, pt1)
#    c = center(pt0, pt1)
#
#    if d > 5:
#        stack.append( (pt0, c) )
#    else:
#        print d
#        L.append(c)
#L.append(pt1)
#print L
#
#print pt1
#
#x = [pt.x for pt in L]
#y = [pt.y for pt in L]
#print "t=", x
#print "s=", y
#print "plot(t,s)"
