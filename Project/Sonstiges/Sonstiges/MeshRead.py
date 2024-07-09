import pyvista as pv
import stl
import numpy

#mesh = pv.read("D:\Studium_Ingenieurinformatik\Produktionsinformatik\Auswahl\Bauteil3_002.bmp")


stlMesh = pv.read('D:\Studium_Ingenieurinformatik\Produktionsinformatik\STL_Testdateien\Rechteck_regulaer.stl')
print(stlMesh)
data = stlMesh.extract_surface()
data.plot()
print(data.face_normals)
print(data.point_normals)



# Using an existing stl file:
your_mesh = stl.Mesh.from_file('D:\Studium_Ingenieurinformatik\Produktionsinformatik\STL_Testdateien\Rechteck_regulaer.stl')
norm = your_mesh.normals
print("Normals: ", norm)
#print("Points", your_mesh.points[0])
print("\n\n")
triangles = []
normals = []
for i in range(len(your_mesh.points)):
    triangles.append([list(your_mesh.points[i][0:3]), list(your_mesh.points[i][3:6]), list(your_mesh.points[i][6:9])])
    normals.append(your_mesh.normals[i])
print("Triangle Data: ", triangles)

# Or creating a new mesh (make sure not to overwrite the `mesh` import by
# naming it `mesh`):
#VERTICE_COUNT = 100
#data = numpy.zeros(VERTICE_COUNT, dtype=stl.Mesh.dtype)
#your_mesh = stl.Mesh(data, remove_empty_areas=False)


