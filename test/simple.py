from mesher.mesh import Mesh, Vertex
from mesher.utils import MeshTester, MeshVisualizer

v0 = Vertex(  0, 0)
v1 = Vertex( 10, 0)
v2 = Vertex(  5, 2)
v3 = Vertex(  5, 5)
v4 = Vertex(8.5, 1)

mesh = Mesh()

mesh.insert(v0)
mesh.insert(v1)
mesh.insert(v2)
mesh.insert(v3)
mesh.insert(v4)

mesh.add_constraint(v0, v1)
mesh.add_constraint(v2, v1)
mesh.add_constraint(v2, v0)

tester = MeshTester(mesh)
tester.audit()
