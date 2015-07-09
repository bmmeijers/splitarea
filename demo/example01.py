from random import randint, seed
from collections import defaultdict, namedtuple

# from mesher.mesh import Mesh, Vertex
# from mesher.utils import MeshVisualizer

from simplegeom.geometry import LineString, Envelope, Point
from simplegeom.wkt import loads
    
from splitarea.flagging import EdgeEdgeHarvester#, MidpointHarvester
from splitarea.skeleton import SkeletonGraph

from tri import triangulate, ToPointsAndSegments # polygon_as_points_and_segments
from tri.delaunay import output_triangles, output_vertices
from tri.delaunay import TriangleIterator, InteriorTriangleIterator, ConvexHullTriangleIterator
from pprint import pprint
import sys

def test():
    
#    from brep.io import geom_from_text

    VertexInfo = namedtuple("VertexInfo", "type face_ids vertex_id")

    wkt = """
    POLYGON ((0 0, 9 1, 10 10, 1 9, 0 0))
    """
    poly = loads(wkt)
    print poly

    conv = ToPointsAndSegments()
    conv.add_point((0,0), info = VertexInfo(0, None, None))
    conv.add_point((9,1), info = VertexInfo(1, None, 1001))
    conv.add_point((10,10), info = VertexInfo(0, None, None))
    conv.add_point((1,9), info = VertexInfo(1, None, 1002))
    conv.add_point((0,0), info = VertexInfo(0, None, None))

    conv.add_segment((0,0), (9,1))
    conv.add_segment((9,1), (10,10))
    conv.add_segment((10,10), (1,9))
    conv.add_segment((1,9), (0,0))

    points, segments, infos = conv.points, conv.segments, conv.infos

    pprint(points)
    pprint(segments)
    pprint(infos)

    dt = triangulate(points, infos, segments)

    for vertex in dt.vertices:
        print vertex, vertex.info

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

    visitor.pick_connectors()



#     visitor = MidpointHarvester([t for t in InteriorTriangleIterator(dt)])
#     visitor.skeleton_segments()
#     with open("/tmp/skel.wkt", "w") as fh:
#         fh.write("wkt\n")
#         for seg in visitor.segments:
#             fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

#     with open("/tmp/centres.wkt", "w") as fh:
#         fh.write("wkt\n")
#         for t, point in visitor.triangle_point.iteritems():
#             fh.write("POINT({0})\n".format(point))

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

    outside_edges = [
        #eid, geom, sn, sn_ext, svtxid, en, en_ext, evtxid, lf, rf
        (4000,
         LineString([(0,20), (1,9)]),
         5002, True, (0,20),
         1002, False, (1,9),

         10000,
         10005
         ),
        (4001,
         LineString([(20,1), (9,1)]),

         5001, True, (20,1),
         1001, False, (9,1),

         10005,
         10000
         ),
    ] 
    
#     outside_edges = []
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
#     print """
#     
#     BRIDGE CONNECTORS
#     """
#     # add nodes from inner rings ("bridge" connectors)
#     for i, segment in enumerate(visitor.ext_segments):
#         v0, v1, = segment
#         ln = LineString()
#         ln.append(Point(*v0.point))
#         ln.append(Point(*v1.point))
#         skeleton.add_segment(ln, 
#                              external = True, 
#                              edge_id = i,
#                              start_vertex_id = v0.gid,
#                              end_vertex_id = v1.gid,
#                              left_face_id = v0.label,
#                              right_face_id = v0.label,
# #                             start_external = True,
# #                             end_external = True
#                              )
# 
#     print """
#     
#     UNLABELED EDGES
#     """
#     # then add all segments which are unlabeled
    for i, segment in enumerate(visitor.segments):
        v0, v1, = segment
        ln = LineString()
        ln.append(Point(v0.x, v0.y))
        ln.append(Point(v1.x, v1.y))
        skeleton.add_segment(ln, 
                             start_vertex_id = (v0.x, v0.y),
                             end_vertex_id = (v1.x, v1.y),
                             external = False, 
                             edge_id = i + len(visitor.ext_segments))

    with open('/tmp/edges.wkt', 'w') as fh:
        fh.write("geometry;leftface;rightface;sn;en\n")
        for edge in skeleton.edges:
            print >> fh, edge.geometry,";", edge.left_face_id,";", edge.right_face_id, ";", edge.start_node.id,";", edge.end_node.id

    skeleton.visualize_nodes()
    skeleton.label_sides()
    skeleton.prune_branches()
    skeleton.find_new_edges()

    print """
    
SKELETON EDGES
    """

    print skeleton.new_edges

    with open("/tmp/edges_new.wkt", "w") as fh:
        fh.write("eid;sn;en;lf;rf;length;geom\n")
        for eid, sn, en, lf, rf, length, geom, in skeleton.new_edges:
            print >> fh, eid, ";", sn,";",  en, ";", lf, ";", rf, ";", length,";",  geom

if __name__ == '__main__':
    test()
