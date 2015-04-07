from tri import triangulate
from tri.delaunay import output_triangles, output_vertices
from tri.delaunay import TriangleIterator, InteriorTriangleIterator, ConvexHullTriangleIterator
from splitarea.flagging import EdgeEdgeHarvester#, MidpointHarvester

def test_poly():
    from connection import connection
    db = connection(True)

    def polygon_input(lines):
        points = []
        segments = []
        points_idx = {}
        for line in lines:
            for pt in line:
                if pt not in points_idx:
                    points_idx[pt] = len(points)
                    points.append(pt)
            for start, end in zip(line[:-1], line[1:]):
                segments.append((points_idx[start], points_idx[end]))
        return points, segments

    lines = []
    sql = 'select geometry from clc_edge where left_face_id in (45347) or right_face_id in (45347)'
    #sql = 'select geometry from clc_edge where left_face_id in (28875) or right_face_id in (28875)'
    # 45270
    sql = 'select geometry from clc_edge where left_face_id in (45270) or right_face_id in (45270)'
    for geom, in db.recordset(sql):
        lines.append(geom)
    points, segments = polygon_input(lines)
    dt = triangulate(points, segments)
    #
    if False:
        trafo = VoronoiTransformer(dt)
        trafo.transform()
        with open("/tmp/centers.wkt", "w") as fh:
            fh.write("wkt\n")
            for incenter in trafo.centers.itervalues():
                fh.write("POINT({0[0]} {0[1]})\n".format(incenter))
        with open("/tmp/segments.wkt", "w") as fh:
            fh.write("wkt\n")
            for segment in trafo.segments:
                # FIXME: why are some not coming through?
                try:
                    fh.write("LINESTRING({0[0]} {0[1]}, {1[0]} {1[1]})\n".format(trafo.centers[segment[0]],
                                                                             trafo.centers[segment[1]]))
                except:
                    pass
    if True:
        with open("/tmp/alltris.wkt", "w") as fh:
                    output_triangles([t for t in TriangleIterator(dt, 
                                                                  finite_only=False)], 
                                     fh)
        with open("/tmp/allvertices.wkt", "w") as fh:
            output_vertices(dt.vertices, fh)
        with open("/tmp/interiortris.wkt", "w") as fh:
                    output_triangles([t for t in InteriorTriangleIterator(dt)], fh)

        with open("/tmp/skelee.wkt", "w") as fh:
            visitor = EdgeEdgeHarvester([t for t in InteriorTriangleIterator(dt)])
            visitor.skeleton_segments()
            fh.write("wkt\n")
            for seg in visitor.segments:
                fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))

        with open("/tmp/skelmp.wkt", "w") as fh:
            visitor = MidpointHarvester([t for t in InteriorTriangleIterator(dt)])
            visitor.skeleton_segments()
            fh.write("wkt\n")
            for seg in visitor.segments:
                fh.write("LINESTRING({0[0].x} {0[0].y}, {0[1].x} {0[1].y})\n".format(seg))


test_poly()