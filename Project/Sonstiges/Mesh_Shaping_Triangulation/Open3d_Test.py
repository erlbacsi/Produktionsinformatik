import open3d as o3d
import pyvista as pv
import numpy as np
import math

def bpa(pcd):
    distances = pcd.compute_nearest_neighbor_distance()
    print("Distances ", distances)
    avg_dist = np.mean(distances)
    print(avg_dist)
    radius = avg_dist * 3

    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius*2, radius*2]))
    #dec_mesh = o3d.geometry.simplify_qadric_decimation(bpa_mesh, 100000)
    #dec_mesh.remove_duplicated_triangles()
    #dec_mesh.remove_duplicated_vertices()
    return bpa_mesh

def compute_alpha(data, slices, zDist=1):    #zwischen den Schichten der Slices
        size = (slices - 1) * zDist
        maxData = []
        for p in range(len(data)):
            zStart = data[p][2] + zDist
            if zStart > size:
                break
            point = data[p]
            minLaenge = float('inf')
            for pz in range(len(data) - 1):
                zSearch = data[pz][2]
                if zSearch == zStart:         #Wenn passender z-Wert gefunden
                    vektor = np.subtract(data[pz], point)
                    print("Vektor1: ", data[pz], " Vektor2: ", point, " gesamt: ", vektor)
                    laenge = math.sqrt(sum([x**2 for x in vektor]))
                    print(laenge)
                    if minLaenge > laenge:
                        minLaenge = laenge
                    if data[pz + 1][2] != zStart or pz + 1 == len(data) - 1:  #Prüfe ob nächster Vektor neuen z-Wert hat oder alle Punkte p getestet wurden
                        maxData.append(minLaenge)
                        break
                if zSearch > zStart:
                    break
        print(maxData)
        return max(maxData)



data = np.array([[0, 0, 0],
                [4, 0, 0],
                [0, 5, 0],
                [4, 5, 0],
                [0, 0, 3],
                [4, 0, 3],
                [0, 5, 3],
                [4, 5, 3]])
normals = np.array([[1, 0, 0],
                    [1, 0, 0],
                    [1, -0, 0],
                    [1, -0, 0]])
color = np.array([[0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0]])

a = compute_alpha(list(data), 2, zDist=3)
print(a)

#arr = np.loadtxt(data)
print(data)

points = pv.PolyData(data)
data = points.delaunay_3d(alpha=3) #REchne mit int Werten -> Größte Strecke ist 6 lang (genau 6,4) -> Radius 3 längste Kante
#data = data.extract_geometry()
#print(data.point_normals)
plotter = pv.Plotter()
#mesh.set_active_vectors("Normals")
#plotter.add_mesh(mesh.arrows, color="red")
plotter.add_mesh(data, show_edges=True, line_width=6)
plotter.show()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(data.points))
pcd.normals = o3d.utility.Vector3dVector(np.array(data.point_normals))
#pcd.colors = o3d.utility.Vector3dVector(color)
#pcd.normals = o3d.utility.Vector3dVector(normals)
#o3d.visualization.draw_geometries([pcd], point_show_normal=True)

o3d.visualization.draw_geometries([pcd], point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=True)

bild = bpa(pcd)
print(np.asarray(bild.triangle_normals))

o3d.visualization.draw_geometries([bild], point_show_normal=True)

#data = np.asarray(pcd.points)
#print(data)




