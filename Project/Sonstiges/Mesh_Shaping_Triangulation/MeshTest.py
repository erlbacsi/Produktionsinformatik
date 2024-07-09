import pyvista as pv
import numpy as np
import matplotlib.pyplot as plt
import math

def norm_vektors(arr):
    i = 0
    for i in len(arr):
        p1 = arr[i]


arr = np.array([[0, 0, 0],
                [0, 4, 0],
                [5, 0, 0],
                [5, 4, 0],
                [0, 0, 3],
                [0, 4, 3],
                [5, 0, 3],
                [5, 4, 3]])

arr2 = np.array([[4, 0, 0],
                [3, 9, 0],
                [7, 3, 0],
                [1, 4, 0],
                [6, 9, 0],
                [4, 6, 0]])

def pointcloud_to_triangles(cloud, zDist=1):
    count = len(cloud)
    triangles = []
    normals = []
    zMax = cloud[count - 1][2]
    for p in range(count - 1):
        # x = Zeile
        startPoint = cloud[p]
        x = startPoint[0]
        z = startPoint[2]
        zStart = startPoint[2] + zDist  # z-Wert der nächsten Ebene
        print(zStart)
        if z == zMax:
            break  # wenn größter z-Wert erreicht abbrechen da keine Ebene mehr darüber liegt

        nextPoint = cloud[p + 1]
        pz = nextPoint[2]
        px = nextPoint[0]
        if px != x:
            min = float('inf')
            # NextWert optimal setzen
            nearPoints = [p for p in cloud if p[0] == px and p[2] == pz]  # Suche alle Werte in dieser Zeile (neuer x-Wert) auf Ebene z-> son diesen kleinsten Abstand bestimmen
            for p in range(len(nearPoints)):
                dist = np.subtract(nearPoints[p], startPoint)
                laenge = math.sqrt(sum([x ** 2 for x in dist]))
                if laenge < min:
                    min = laenge
                    optPoint = nearPoints[p]
            nextPoint = optPoint
            print("Point:", nextPoint)
        if pz != z:
            continue  # mache hier bei Wechsel einfach mit Schleifendurchlauf weiter damit startPoint der Punkt mit neuem z-Wert ist
        minLaenge = float('inf')
        for a in range(p, count):  # Suche Punkte auf nächster Ebene mit passendem z-Wert
            pAbove = cloud[a]
            zSearch = pAbove[2]
            if zSearch == zStart:  # Wenn passender z-Wert gefunden dann kalkuliere ob gefundene Punkte kleinstes aktuelles Dreieck aufspannen
                vektor1 = np.subtract(pAbove, startPoint)
                vektor2 = np.subtract(pAbove, nextPoint)
                laenge1 = math.sqrt(sum([x ** 2 for x in vektor1]))
                laenge2 = math.sqrt(sum([x ** 2 for x in vektor2]))
                lgesamt = laenge1 + laenge2
                if minLaenge > lgesamt:
                    minLaenge = lgesamt
                    pTop = pAbove
            if zSearch > zStart or a + 1 == count - 1:
                triangles.append([startPoint, nextPoint, pTop])
                normals.append(np.cross(vektor1, vektor2))
                break
    return triangles, normals


def plot_triangles(triangles):
    print("Plot Triangles Funktion")
    fig = plt.figure(figsize=(10, 10))
    p = fig.add_subplot(111, projection="3d")
    for i in range(len(triangles)):
        print(triangles[i])
        p.plot(triangles[i][0], triangles[i][1], '-')
        p.plot(triangles[i][0], triangles[i][2], '-')
        p.plot(triangles[i][1], triangles[i][2], '-')
    plt.show()

def plot_triangles_2(triangles):
    fig = plt.figure(figsize=(10, 10))
    p = fig.add_subplot(111, projection="3d")
    for i in range(len(triangles)):
        p.plot(triangles[i][:,0], triangles[i][:,1], triangles[i][:,2], '-')
        plt.show()

triangles, normals = pointcloud_to_triangles(arr, 3)
print("Dreiecke: ", triangles)
print("Normalen: ", normals)
plot_triangles(triangles)

s = []
s = [p for p in arr if p[1] == 4 and p[2] == 3]
print(s)
print(s[1][1])



cloud = pv.PolyData(arr2)
#cloud.plot()

volume = cloud.delaunay_2d()
print(volume.faces)
#volume = cloud.reconstruct_surface(nbr_sz=5)
mesh = volume.extract_surface()
faces = mesh.faces.reshape((-1, 4))
print(faces)
faces = faces[:, 1:]
a = mesh.points[faces]
print(a)
#plot_triangles_2(a)


'''print(mesh.point_normals)
print(mesh.face_normals)
updateMesh = mesh.compute_normals(cell_normals=True, point_normals=True, progress_bar=True)'''

'''mesh.save("D:\Studium_Ingenieurinformatik\Produktionsinformatik\GitLab_Project\produktionsinformatik\Project\Spielwiese\Sonstiges\Wuerfel.stl", binary=False)
mesh.plot(show_bounds=True)
print(updateMesh.face_normals)

plotter = pv.Plotter()
updateMesh.set_active_vectors("Normals")
plotter.add_mesh(updateMesh.arrows, color="red")
plotter.add_mesh(updateMesh, show_edges=True, line_width=6)
plotter.show()
#updateMesh.plot_normals()'''

