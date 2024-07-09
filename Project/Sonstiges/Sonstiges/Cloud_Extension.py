'''
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from Directory import Directory
from BMP import BMP
import TimeMeasure
import pyvista as pv
import math
import stl

#-------------------------------------------------------------------------------------------------
#------Funktionen und Algorithmen zur erezugung eines vollständigen Meshes aus einer Punktewolke--
#-------------------------------------------------------------------------------------------------

    def pointcloud_to_triangles(self, cloud, zDist=1):
        count = len(cloud)
        triangles = []
        normals = []
        zMax = cloud[count-1][2]
        for p in range(count-1):
            #x = Zeile
            startPoint = cloud[p]
            x = startPoint[0]
            z = startPoint[2]
            zStart = startPoint[2] + zDist  #z-Wert der nächsten Ebene
            if zStart == zMax:
                break  #wenn größter z-Wert erreicht abbrechen da keine Ebene mehr darüber liegt

            nextPoint = cloud[p+1]
            pz = nextPoint[2]
            px = nextPoint[0]
            if px != x:
                min = float('inf')
                #NextWert optimal setzen
                nearPoints = [p for p in cloud if p[0] == px and p[2] == pz] #Suche alle Werte in dieser Zeile (neuer x-Wert) auf Ebene z-> son diesen kleinsten Abstand bestimmen
                for p in range(nearPoints):
                    dist = np.subtract(p, startPoint)
                    if dist < min:
                        min = dist
                        optPoint = p
                nextPoint = optPoint
            if pz != z:
                continue  #mache hier bei Wechsel einfach mit Schleifendurchlauf weiter damit startPoint der Punkt mit neuem z-Wert ist
            minLaenge = float('inf')
            for a in range(p, count):    #Suche Punkte auf nächster Ebene mit passendem z-Wert
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
                if zSearch > zStart:
                    triangles.append([startPoint, nextPoint, pTop])
                    normals.append(np.cross(vektor1, vektor2))
                    break
        return triangles, normals

    def plot_triangles(self, triangles):
        fig = plt.figure(figsize=(10, 10))
        p = fig.add_subplot(111, projection="3d")
        for i in range(len(triangles)):
            p.plot(triangles[i][:,0], triangles[i][:,1], triangles[i][:,2], '-')
            plt.show()

    def stl_to_pointcloud(self, path):
        stlMesh = pv.read(path)
        data = stlMesh.extract_surface()
        return data, data.points                     #return PolyData Objekt und Punktewolke als numpyArray -> enthält Punkte, und Normalenvektoren

    def stl_face_normals(self, data):           #benötigt PolyData Objekt
        return data.face_normals

    def stl_point_normals(self, data):           #benötigt PolyData Objekt
        return data.point_normals

    def pointcloud_to_mesh(self, pointcloud, saving_path, dateiname):   #Gibt Polydata Objekt zurück
        data = pv.PolyData(pointcloud)
        print("Polydata Umwandlung abgeschlossen")
        a = self.compute_alpha(pointcloud, len(self.dateien), self.downsampleSize, self.zAbstand)
        volume = data.delaunay_3d(alpha=a, progress_bar=True)   #Triangulierung
        print("Triangulierung abgeschlossen")
        mesh = volume.extract_geometry()  #Flächen erzeugen
        print("Flächen erzuegen abgeschlossen")
        mesh = mesh.compute_normals(cell_normals=True, point_normals=True, progress_bar=True)
        print("Normalen nachberechnen abgeschlossen")
        mesh.save(saving_path + dateiname + ".stl", binary=False)
        print("STL speicher abgeschlossen")
        #Erzeugtes Mesh plotten
        plotter = pv.Plotter()
        #mesh.set_active_vectors("Normals")
        #plotter.add_mesh(mesh.arrows, color="red")
        plotter.add_mesh(volume, show_edges=True, line_width=6)
        plotter.show()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(mesh.points))
        pcd.normals = o3d.utility.Vector3dVector(np.array(mesh.point_normals))
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

        pivot = self.bpa(pcd)
        pivot_mesh = np.asarray(pivot.triangle_normals)
        print(len(pivot_mesh))

        o3d.visualization.draw_geometries([pivot], mesh_show_wireframe=True, mesh_show_back_face=True)
        return mesh

    def mesh_point_normals(self, mesh):         #benötigt PolyData Objekt
        return mesh.point_normals

    def mesh_face_normals(self, mesh):           #benötigt PolyData Objekt
        return mesh.face_normals

    def bpa(self, pcd):
        distances = pcd.compute_nearest_neighbor_distance()
        print(distances)
        avg_dist = np.mean(distances)
        print(avg_dist)
        radius = avg_dist * 2

        bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius*2, radius*4]))
        # dec_mesh = o3d.geometry.simplify_qadric_decimation(bpa_mesh, 100000)
        # dec_mesh.remove_duplicated_triangles()
        # dec_mesh.remove_duplicated_vertices()
        return bpa_mesh

    def compute_alpha(self, data, slices, sampleFactor, zDist=1):  # zwischen den Schichten der Slices
        size = (slices - 1) * zDist
        minData = []
        for p in range(len(data)):
            zStart = data[p][2] + zDist
            if zStart > size:
                break
            point = data[p]
            minLaenge = float('inf')
            for pz in range(len(data) - 1):
                zSearch = data[pz][2]
                if zSearch == zStart:  #Wenn passender z-Wert gefunden
                    vektor = np.subtract(data[pz], point)
                    laenge = math.sqrt(sum([x ** 2 for x in vektor]))
                    if minLaenge > laenge:
                        minLaenge = laenge
                    if data[pz + 1][2] != zStart or pz + 1 == len(data) - 1:  # Prüfe ob nächster Vektor neuen z-Wert hat oder alle Punkte p getestet wurden
                        minData.append(minLaenge)
                        break
                if zSearch > zStart:
                    break
        print(minData)
        return min(minData) * (2 * zDist) * sampleFactor'''