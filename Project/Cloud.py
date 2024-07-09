import numpy as np
import open3d as o3d
from Directory import Directory
from BMP import BMP
import TimeMeasure
import stl


class StlReader:
    def __init__(self):
        self.triangles = []
        self.normals = []

    # Einlesen der STL Datei und Rückgabe der Dreiecke und Normalen in einer Liste
    # Eingabe des Dateinamens und des Pfades in die Funktion
    def readMesh(self, dateiname):
        stl_points = []
        mesh = stl.Mesh.from_file(dateiname)
        # Finde größten wert und teile alle durch diesen -> Objekt skalieren
        for i in range(len(mesh.points)):
            #print("meshPoints", mesh.points[i][0:3])
            stl_points.append(mesh.points[i][0:3])
            stl_points.append(mesh.points[i][3:6])
            stl_points.append(mesh.points[i][6:9])
        #print("Mesh: ", stl_points)
        werte = np.hstack(mesh.points)
        #print("Werte: ", werte)
        maxWert = max(werte)
        #print("Max Wert", maxWert)
        werteSkaliert = mesh.points / maxWert
        for i in range(len(mesh.points)):
            self.triangles.append([list(werteSkaliert[i][0:3]), list(werteSkaliert[i][3:6]), list(werteSkaliert[i][6:9])])
            self.normals.append(mesh.normals[i])
        #print("Triangles skaliert: ", self.triangles)
        return self.triangles, self.normals, maxWert, stl_points


class Punktewolke:

    def __init__(self, path="", zAbstand=1, downsampleSize=3):
        self.directory = Directory(path)                 #erzeuge Directory Objekt mit Path zu Bitmapdateien
        self.dateien, self.pfad = self.directory.data_name_list()   #Schreibe alle Dateinamen in eine Liste dateien und den Pfad dazu
        self.zAbstand = zAbstand
        self.downsampleSize = downsampleSize
        #print(self.dateien)

    def pointcloud(self, visualize: bool):
        i = 0                                           #zählt Anzahl der Punkte
        number = 0                                      #zählt Anzahl der Punkte die in Punktewolke kommen
        d = 0                                           #z-Achse Abstand

        #time = TimeMeasure.TimeMeasurement()

        for z in range(len(self.dateien)):                 #Gehe alle Bilder durch
            #time.start()
            bmp = BMP(self.pfad, self.dateien[z])     #erzuege BMP Instanz mit entsprechendem Dateinamen
            image = bmp.bmp_to_array()                 #Erhalte Array der Bilddaten
            if (z == 0):                                   #An dem ersten Bild die Höhe und Weite auslesen
                width = image.shape[1]
                height = image.shape[0]
                print("width, height", width, height)
                wolke = []
            print("Datei: ", self.dateien[z], " wird berechnet")
            for x in range(height):
                for y in range(width):
                    b = image[x][y][0]
                    g = image[x][y][1]
                    r = image[x][y][2]
                    if b == 255 and g == 255 and r == 255:
                        # Koordinaten direkt umwandeln von Pixelkoordinaten in Punktkoordinaten -> durch 10 da ein 10 Pixel einem Millimeter entsprechen (Slicer Einstellung)
                        wolke.append([x / 10, y / 10, d])
                        number += 1
                        i += 1
            d += self.zAbstand
            if d == 1:
                print("wolke", wolke)
            #time.end("Berechnung Bild {}".format(z))

        #time.closeData()
        print("Anzahl Punkte: ", number)
        wolke, new_points = self.downsample_list(wolke, number, self.downsampleSize)
        print("Anzahl dezimierter Punkte: ", new_points)
        #Visualisierung
        if visualize:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(wolke))
            o3d.visualization.draw_geometries([pcd], point_show_normal=True)
        return wolke

    #Anzahl Punkte in der Punktewolke reduzieren
    #übernehme jeden num_points Punkt -> num_points = 3 heißt es wird jeder dritte Punkt übernommen
    def downsample_list(self, points, num_points, teiler):
        decimated = []
        new_points = 0
        for i in range(num_points):
            if i % teiler == 0:
                decimated.append(points[i])
                new_points += 1
        return decimated, new_points



