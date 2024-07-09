import Cloud
import Shaping
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import stl
import TimeMeasure

time = TimeMeasure.TimeMeasurement()

time.start()
print("Starte den Prozess...")

#------------------------------------------------------#
#---------------- Path to STL data ----------------------#
#path = "D:\Ingenieurinformatik\Produktionsinformatik\Testdateien\STL_rotiert\\120Grad\Wuerfel"
path = "path/to/sliced/data"

# Ordner mit den STL Dateien, welche die Zieltransformation enthalten
#path_stl = "D:\Ingenieurinformatik\Produktionsinformatik\Testdateien\STL_rotiert\\120Grad\Wuerfel.stl"
path_stl = "path/to/stl/data"
#--------------------------------------------------------#

# Behalte jedes n-te Pixel (z.B. downsample = 3 -> Behalte jedes dritte Pixel) um die Punktewolke zu verkleinern
downsample = 1

# Erzeuge Punktewolke aus Bitmaps
p = Cloud.Punktewolke(path, 1, downsample)
wolke = p.pointcloud(visualize=True)
print("Erzeugen der Punktewolke beendet")

# Erzeuge uniques Dreieck in der Punktewolke und visualisiere diese
shapeWolke = Shaping.Shaping()
plotCloud, triangle, triangle_index, normals, uniqueData, randomTrPoints = shapeWolke.objectEstimation(wolke, 3, downsample, True)
shapeWolke.plot_edges(plotCloud, triangle_index, False)
print("Erzeugen des uniquen Triangles beendet")

# Erzeuge nun in der STL Datei viele zufällige unique Dreiecke
# Suche das passende Dreieck zu dem in der Punktewolke und berechne die Transformation zwischen den Dreiecken
shapeSTL = Shaping.Shaping()
rotation, translation, wolkePoints, wolkeCentre = shapeSTL.objectTransformation(path_stl, wolke, uniqueData, randomTrPoints, testCases=500000)

time.end("Transformation beendet")
time.closeData()

# Berechne die gedrehte STL-Datei
wolkeRotated = rotation @ np.transpose(wolkePoints)
# Berechne Translation auf der STL-Datei -> STL und Punkteolke überlappen
wolkeTranslated = np.array(wolkeRotated) + translation
print("Transformation berechnet...")

#------------------------------------------------------------------------------------
#-------------------------- Visualisere die Ergebnisse ------------------------------
#------------------------------------------------------------------------------------
# Ausgangssituation
x = [p[0] for p in wolke]
y = [p[1] for p in wolke]
z = [p[2] for p in wolke]
fig = plt.figure(figsize=(10, 10), num="Ausgangssituation")
#axes = fig.add_subplot(111, projection="3d")
axes = mplot3d.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(axes)
m1 = stl.Mesh.from_file("D:\Ingenieurinformatik\Produktionsinformatik\Testdateien\Wuerfel_original.stl")
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(m1.vectors))
axes.scatter(x, y, z, c="red")
axes.set_zlim3d(-5, 20)
axes.set_xlim3d(-10, 80)
axes.set_ylim3d(-10, 80)
plt.show()


# Translation to (0, 0, 0)
x = [p[0] for p in wolke]
y = [p[1] for p in wolke]
z = [p[2] for p in wolke]

wolkeSTL = []
wolkeReshaped = np.array(wolkePoints).reshape(-1, 3)
le = int(len(wolkeReshaped) / 3)
i = 0
j = 0
while j < le:
    wolkeSTL.append([wolkeReshaped[i], wolkeReshaped[i + 1], wolkeReshaped[i + 2]])
    i = i + 3
    j = j + 1

fig = plt.figure(figsize=(10, 10), num="Translation auf den Koordinatenursprung")
axes = mplot3d.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(axes)
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(wolkeSTL))
axes.scatter(x, y, z, c="red")
axes.set_zlim3d(-5, 20)
axes.set_xlim3d(-10, 80)
axes.set_ylim3d(-10, 80)
plt.show()

# Rotation
x = [p[0] for p in wolke]
y = [p[1] for p in wolke]
z = [p[2] for p in wolke]

wolkeSTL = []
wolkeReshaped = wolkeRotated.T
le = int(len(wolkeReshaped) / 3)
i = 0
j = 0
while j < le:
    wolkeSTL.append([wolkeReshaped[i], wolkeReshaped[i + 1], wolkeReshaped[i + 2]])
    i = i + 3
    j = j + 1

fig = plt.figure(figsize=(10, 10), num="Rotation")
axes = mplot3d.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(axes)
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(wolkeSTL))
axes.scatter(x, y, z, c="red")
axes.set_zlim3d(-5, 20)
axes.set_xlim3d(-10, 80)
axes.set_ylim3d(-10, 80)
plt.show()

# Translation
x = [p[0] for p in wolke]
y = [p[1] for p in wolke]
z = [p[2] for p in wolke]

wolkeSTL = []
wolkeReshaped = wolkeTranslated.T
le = int(len(wolkeReshaped) / 3)
i = 0
j = 0
while j < le:
    wolkeSTL.append([wolkeReshaped[i], wolkeReshaped[i + 1], wolkeReshaped[i + 2]])
    i = i + 3
    j = j + 1

fig = plt.figure(figsize=(10, 10), num="Translation auf Zielposition")
axes = mplot3d.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(axes)
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(wolkeSTL))
axes.scatter(x, y, z, c="red")
axes.set_zlim3d(-5, 20)
axes.set_xlim3d(-10, 80)
axes.set_ylim3d(-10, 80)
plt.show()


