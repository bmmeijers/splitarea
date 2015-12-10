from random import randint, seed
from collections import defaultdict

# from mesher.mesh import Mesh, Vertex
# from mesher.utils import MeshVisualizer

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads
    
from splitarea.flagging import TriangleVisitor , EdgeEdgeHarvester, MidpointHarvester
from splitarea.skeleton import SkeletonGraph

from tri import triangulate, polygon_as_points_and_segments
from tri.delaunay import output_triangles, output_vertices
from tri.delaunay import TriangleIterator, InteriorTriangleIterator, ConvexHullTriangleIterator
from pprint import pprint
import sys

def test():
    
#    from brep.io import geom_from_text
    
    wkt = """
    POLYGON ((1748051.29733583 5136615.82947196 773.937, 1748047.48 5136606.14 774.133, 1748046.06 5136603.88 773.969, 1748042.35 5136597.96 773.537, 1748047.67 5136593.68 773.537, 1748062.48 5136584.98 773.537, 1748067.52 5136582.04 773.537, 1748078.22 5136575.79 773.537, 1748087.88 5136571.84 773.537, 1748100.04 5136568.43 773.537, 1748114.52 5136567.96 773.537, 1748123.7 5136568.71 773.537, 1748126.1 5136568.9 773.537, 1748139.61 5136568.29 773.537, 1748149.57 5136565.95 773.537, 1748151.93 5136555.15 773.537, 1748149.21 5136548.6 773.537, 1748145.32 5136546.63 773.537, 1748142.02 5136544.95 773.537, 1748131.75 5136545.4 773.537, 1748127.61 5136546.26 773.537, 1748103.12 5136551.32 773.537, 1748099.8 5136552.01 773.537, 1748097.94 5136550.26 773.537, 1748104.5 5136544.95 773.537, 1748110.82 5136539.96 773.537, 1748115.85 5136536 773.537, 1748120.75 5136530 773.537, 1748122.84 5136527.43 773.537, 1748130.56 5136525.53 773.529, 1748137.16 5136523.9 773.522, 1748181.68 5136506.95
773.582, 1748242.81 5136487.67 772.94, 1748259.51 5136493.28 772.94, 1748258.84 5136498.26 772.94, 1748301.42 5136484.25 772.94, 1748297.81 5136473.28 771.782, 1748340.02 5136463.62 771.782, 1748354.35 5136457.53 771.782, 1748368.83 5136450.68 771.782, 1748369.05 5136450.98 771.788, 1748372.99 5136456.65 771.892, 1748377.35 5136461.37 772.002, 1748382.3 5136465.89 772.112, 1748386.44 5136469.71 772.222, 1748392.7 5136473.98 772.332, 1748394.36 5136475.02 772.362, 1748398.85 5136477.82 772.442, 1748404.63 5136480.54 772.552, 1748412.49 5136483.25 772.662, 1748419.53 5136484.98 772.772, 1748426.23 5136486.04 772.882, 1748433.03 5136486.76 772.992, 1748441.07 5136486.75 773.102, 1748448.67 5136486.1 773.212, 1748454.72 5136485.04 773.342, 1748455.57 5136484.81 773.342, 1748455.03 5136493.07 773.342, 1748451.87 5136500.92 773.342, 1748448.06 5136508.98 773.342, 1748443.03 5136515.29 773.342, 1748435.89 5136523.68 773.342, 1748426.88 5136534.43 773.342, 1748419.43 5136547.01 773.
342, 1748412.45 5136556.31 773.342, 1748403.46 5136567.93 773.342, 1748395.46 5136580.18 773.342, 1748392.94 5136593.95 773.342, 1748395.02 5136601.5 773.342, 1748393.12 5136622.45 773.342, 1748390.53 5136630.67 773.129, 1748384.75 5136630.93 773.129, 1748376.07 5136635.84 773.129, 1748375.44 5136643.04 773.129, 1748369.59 5136648.43 773.129, 1748367.26 5136649.46 773.129, 1748363.05 5136651.32 773.129, 1748358.76 5136653.21 773.129, 1748356.79 5136656.19 773.129, 1748354.07 5136660.32 773.129, 1748348.53 5136665.18 773.129, 1748333.14 5136668.96 773.129, 1748328.14 5136669.42 773.129, 1748320.61 5136670.1 773.129, 1748315.97 5136670.48 773.129, 1748315.3 5136670.54 773.129, 1748304.63 5136671.4 773.129, 1748292.43 5136678.1 773.129, 1748284.63 5136689.93 773.129, 1748282.24 5136693.79 773.129, 1748278.53 5136699.76 773.129, 1748266.66 5136710.23 773.129, 1748257.66 5136711.9 773.129, 1748252.92 5136710.38 773.129, 1748245.92 5136708.12 773.129, 1748233.48 5136705.31 773.129,
1748223.28 5136703 773.129, 1748211.97 5136693.03 773.129, 1748206.17 5136686 773.114, 1748187.06 5136659.51 773.114, 1748185.87 5136658.15 773.101, 1748181.55 5136653.23 773.054, 1748173.2 5136636.95 773.054, 1748169.79 5136630.29 773.054, 1748157.99 5136613.96 773.054, 1748145.41 5136605.64 773.069, 1748137.18 5136604.48 773.084, 1748133.63 5136604.66 773.084, 1748122.27 5136605.23 773.084, 1748112.05 5136608.23 773.084, 1748096.84 5136610.21 773.084, 1748083.65 5136610.56 773.069, 1748070.37 5136611.7 773.174, 1748054.86 5136614.9 773.384, 1748051.29733583 5136615.82947196 773.937))
    """
    
    poly = loads(wkt)
    
    print poly
    points, segments = polygon_as_points_and_segments(poly)
    pprint(points)
    pprint(segments)
    dt = triangulate(points, segments)
    with open("/tmp/alltris.wkt", "w") as fh:
        output_triangles([t for t in TriangleIterator(dt)], fh)
    with open("/tmp/allvertices.wkt", "w") as fh:
        output_vertices(dt.vertices, fh)
    with open("/tmp/interiortris.wkt", "w") as fh:
        output_triangles([t for t in InteriorTriangleIterator(dt)], fh)
    with open("/tmp/hull.wkt", "w") as fh:
        output_triangles([t for t in ConvexHullTriangleIterator(dt)], fh)

#     #if DEBUG: print poly
#     ln = []
#     for ring in poly:
#         for vtx in ring:
#             ln.append(vtx)
#         
#     ev = poly.envelope
#     #if DEBUG: print ev
#     eps = 10000
#     half_dx = (ev.xmax - ev.xmin) / 2.0
#     dy = (ev.ymax - ev.ymin)
#     # top - middle
#     top_y = ev.ymax + dy + eps
#     top_x = ev.xmin + half_dx
#     # bottom - left
#     left_x = ev.xmin - half_dx - eps
#     left_y = ev.ymin - eps
#     # bottom - right
#     right_x = ev.xmax + half_dx + eps
#     right_y = ev.ymin - eps
#     
#     bnd = [Vertex(left_x,left_y), 
#         Vertex(right_x,right_y), 
#         Vertex(top_x,top_y)]
#     # return
#     mesh = Mesh(boundary = bnd)
# #    mesh = Mesh()
#     prev_pt = None
#     seed("ab")
#     for i, pt in enumerate(ln):
#         vtx = Vertex(pt.x, pt.y)
#         
#         if i == 2:
#             vtx.flag = 1
#             ext_end = Point(pt.x, pt.y)
#             
#         elif i == 60:
#             vtx.flag = 1
#             ext_end2 = Point(pt.x, pt.y)    
#         else:
#             vtx.flag = 0 # int(randint(0, 10) in (5, ))
#         
#         vtx = mesh.insert(vtx)
#         vtx.gid = i
#         if i == 2:
#             ext_end_id = vtx.gid
#         elif i == 60:
#             ext_end2_id = vtx.gid
#         
#         if i > 0:
#             mesh.add_constraint( prev_pt, vtx )
#         prev_pt = vtx
#     
#     fh = open('/tmp/tris.wkt', 'w')
#     fh.write("geometry\n")
#     MeshVisualizer(mesh).list_triangles_wkt(fh)
#     fh.close()

    visitor = EdgeEdgeHarvester([t for t in InteriorTriangleIterator(dt)])
    visitor.skeleton_segments()
    with open("/tmp/skel0.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    visitor = MidpointHarvester([t for t in InteriorTriangleIterator(dt)])
    visitor.skeleton_segments()
    with open("/tmp/skel.wkt", "w") as fh:
        fh.write("wkt\n")
        for seg in visitor.segments:
            fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

    with open("/tmp/centres.wkt", "w") as fh:
        fh.write("wkt\n")
        for t, point in visitor.triangle_point.iteritems():
            fh.write("POINT({0})\n".format(point))
    
#     sys.exit()

#     visitor = TriangleVisitor(dt)
#     visitor.skeleton_segments()
#     visitor.pick_connectors()
# 
#     visitor.list_table_sg()
#     visitor.list_segments_pg()
#     visitor.list_connectors_pg()
#     visitor.list_pg()

#    visualizer = MeshVisualizer(mesh)
#    visualizer.list_pg()
    
    # make artificial external edges
#     ext_start = Point(1747940, 5136656)
#     ln = LineString()
#     ln.append(ext_start)
#     ln.append(ext_end)
#     outside_edges = [
#         (500000, ln, 100, True, -1, 101, False, ext_end_id, 200, 201)] 
#         #(eid, geom, sn, sn_ext, en, en_ext, lf, rf)
#     
#     ext_start2 = Point(1748550, 5136537)
#     ln = LineString()
#     ln.append(ext_start2)
#     ln.append(ext_end2)
#     outside_edges.append((600000, ln, 200, True,  -2, 201, False, ext_end2_id, 201, 200))
#     
    outside_edges = []
    skeleton = SkeletonGraph()
    print """
    
    ADDING OUTSIDE EDGES
    """
    # first add outside edges
    for outside_edge in outside_edges:
        eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf, = outside_edge
        skeleton.add_segment(geom, 
                             external = True, 
                             edge_id = eid, 
                             left_face_id = lf, 
                             right_face_id = rf,
                             start_node_id = sn, 
                             end_node_id = en,
                             start_vertex_id = svtxid,
                             end_vertex_id = evtxid,
                             start_external = sn_ext,
                             end_external = en_ext
                             )
    print """
    
    BRIDGE CONNECTORS
    """
    # add nodes from inner rings ("bridge" connectors)
    for i, segment in enumerate(visitor.ext_segments):
        v0, v1, = segment
        ln = LineString()
        ln.append(Point(*v0.point))
        ln.append(Point(*v1.point))
        skeleton.add_segment(ln, 
                             external = True, 
                             edge_id = i,
                             start_vertex_id = v0.gid,
                             end_vertex_id = v1.gid,
                             left_face_id = v0.label,
                             right_face_id = v0.label,
#                             start_external = True,
#                             end_external = True
                             )

    print """
    
    UNLABELED EDGES
    """
    # then add all segments which are unlabeled
    for i, segment in enumerate(visitor.segments):
        v0, v1, = segment
        ln = LineString()
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        skeleton.add_segment(ln, 
                             start_vertex_id = v0.gid,
                             end_vertex_id = v1.gid,
                             external = False, 
                             edge_id = i + len(visitor.ext_segments))
    
    fh = open('/tmp/edges.wkt', 'w')
    fh.write("geometry;leftface;rightface;sn;en\n")
    for edge in skeleton.edges:
        print >> fh, edge.geometry,";", edge.left_face_id,";", edge.right_face_id, ";", edge.start_node.id,";", edge.end_node.id
    fh.close()
        
    skeleton.visualize_nodes()
    skeleton.label_sides()
    skeleton.prune_branches()
    skeleton.find_new_edges()
    
    fh = open("/tmp/edges_new.wkt", "w")
    fh.write("eid;sn;en;lf;rf;length;geom\n")
    for eid, sn, en, lf, rf, length, geom, in skeleton.new_edges:
        print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", length,";",  geom
    fh.close()

if __name__ == '__main__':
    test()
