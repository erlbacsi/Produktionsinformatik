import numpy as np
import matplotlib.pyplot as plt
import stl
import Cloud
import math
from mpl_toolkits import mplot3d


def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    # if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    t = -R @ centroid_A + centroid_B
    print("Fertige Implementierug:", R)
    print("STL Punkte mit Rotationsmatrix: \n", R @ Am)
    rotated = R @ A + t
    '''a = createPlot()
    pointAdd(rotated.T, a)
    pointAdd(B.T, a)
    plotPoints()'''


def rotationMatching(originalPoints, rotatedPoints):
    # Matrizen mit den Punktewolken, welche in den Ursprung verschoben wurden
    centredOriginalP = []
    centredRotatedP = []
    numberPoints = len(originalPoints)
    originalCentre = mittelpunkt(originalPoints)
    rotatedCentre = mittelpunkt(rotatedPoints)
    for i in range(numberPoints):
        centredOriginalP.append(np.subtract(np.array(originalPoints[i]), originalCentre))
        centredRotatedP.append(np.subtract(np.array(rotatedPoints[i]), rotatedCentre))

    a = createPlot()
    pointAdd(centredOriginalP, a)
    pointAdd(centredRotatedP, a)
    plotPoints()

    # Werden die beiden Matrizen andersrum multipliziert wird die Rotation von RotatedP zu OriginalP berechnet
    h = np.transpose(centredOriginalP) @ centredRotatedP

    u, s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[2, :] *= -1
        r = vt.T @ u.T
    t = -r @ centredOriginalP + centredRotatedP
    print("Eigene Implementierung:", r)
    print("STL Punkte mit Rotationsmatrix: \n", r @ np.transpose(centredOriginalP))
    rotatedA = r @ np.transpose(centredOriginalP)
    print(rotatedA.T)
    a = createPlot()
    pointAdd(rotatedA.T, a)
    pointAdd(centredRotatedP, a)
    plotPoints()
    return r, t

def mittelpunkt(cloud):
    xValues = [p[0] for p in cloud]
    yValues = [p[1] for p in cloud]
    zValues = [p[2] for p in cloud]
    xAverage = sum(xValues) / len(xValues)
    yAverage = sum(yValues) / len(yValues)
    zAverage = sum(zValues) / len(zValues)
    return [xAverage, yAverage, zAverage]


def createPlot():
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")
    return ax

def pointAdd(points, ax):
    x = []
    y = []
    z = []
    for p in points:
        ax.scatter(p[0], p[1], p[2])
        x.append(p[0])
        y.append(p[1])
        z.append(p[2])
    # ersten Punkt hinten nochmal anhängen
    x.append(x[0])
    y.append(y[0])
    z.append(z[0])
    # Punkte müssen unbedingt in Liste liegen und Werte durch Komma getrennt werden -> beim Slicen der Spalten ist das nicht der Fall
    ax.plot(x, y, z)

def plotPoints():
    plt.show()

q = np.array([0, 1, 2])
print(q)
print(q.reshape(-1, 1))


mesh = stl.Mesh.from_file("D:\Studium_Ingenieurinformatik\Produktionsinformatik\STL_Testdateien\Rechteck_regulaer_skaliert.stl")
print("Points ", mesh.vectors)
werte = np.hstack(mesh.points)
werte = mesh.points / max(werte)
print(werte)
print(max(mesh.points[1]))
print("---------")



#A = [[0,0,1], [0,1,0], [0,0,0]]
#B = [[0,0,0], [0,1,0], [0,0,1]]
A = [[51.37887979038538, 47.0, 1.2641118035283099], [45.0, 52.747681787167956, 2.0005387754297037], [48.360117379358044, 47.0, 1.2575256736111482], [51.37887979038538, 47.0, 1.2641118035283099], [45.0, 52.747681787167956, 2.0005387754297037], [48.360117379358044, 47.0, 1.2575256736111482], [51.37887979038538, 47.0, 1.2641118035283099], [45.0, 52.747681787167956, 2.0005387754297037], [48.360117379358044, 47.0, 1.2575256736111482]]
B = [[-0.3294202, 1.0, 0.62798244], [0.6, -0.3604833, 0.77163756], [0.32237968, 0.99999994, 0.6115394]]
wolkeSTL = []
wolkeReshaped = np.array(A).reshape(-1, 3)
print("WOLKE  Form", wolkeReshaped)
l = int(len(wolkeReshaped) / 3)
i = 0
j = 0
while j < l:
    wolkeSTL.append([wolkeReshaped[i], wolkeReshaped[i + 1], wolkeReshaped[i + 2]])
    i = i + 3
    j = j + 1
print("WOLKE STL Form", wolkeSTL)


#rigid_transform_3D(np.transpose(np.array([[1, 1, 3], [2, 1, 3], [1, 2, 3]])), np.transpose(np.array([[2, 4, 3], [2, 5, 3], [3, 5, 3]])))


path = "D:\Studium_Ingenieurinformatik\Produktionsinformatik\Testdateien\STL_tranformiert\90Grad_x=20_y=17\Wuerfel"
path_stl = "D:\Studium_Ingenieurinformatik\Produktionsinformatik\STL_Testdateien\Testing_STL\\"
downsample = 1


# Show the plot to the screen


#erzeuge Punktewolke aus Bitmaps
p = Cloud.Punktewolke(path, 1, downsample)
wolke = p.pointcloud(0.1, visualize=False)

#rotation, t = rotationMatching(A, B)
#print("trans: ", t)
'''
xValues = [p[0] for p in wolke]
yValues = [p[1] for p in wolke]
zValues = [p[2] for p in wolke]
xAverage = sum(xValues) / len(xValues)
yAverage = sum(yValues) / len(yValues)
zAverage = sum(zValues) / len(zValues)
m = [xAverage, yAverage, zAverage]
for i in range(len(wolke)):
    wolke[i] = list(np.array(wolke[i]) - np.array(m))

drehung = np.array([[math.cos(np.radians(90)), -math.sin(np.radians(90)), 0],
                    [math.sin(np.radians(90)), math.cos(np.radians(90)), 0],
                    [0, 0, 1]])
                    '''
#print(drehung)
#wolkeRotated = rotation.T @ np.transpose(wolke)
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
#x = [p[0] for p in wolkeRotated.T]
#y = [p[1] for p in wolkeRotated.T]
#z = [p[2] for p in wolkeRotated.T]
x = [p[0] for p in wolke]
y = [p[1] for p in wolke]
z = [p[2] for p in wolke]
#ax.scatter(x, y, z, c="red")
#ax.scatter(x1, y1, z1, c="green")'''


# Create a new plot
#fig = plt.figure(figsize=(10, 10))
#axes = mplot3d.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)

m1 = stl.Mesh.from_file("D:\Studium_Ingenieurinformatik\Produktionsinformatik\STL_Testdateien\Testing_STL\original_pyramide.stl")
m = [[0, 0,0], [0, 0, 1], [0, 1, 0], [1, 0,0], [1, 0, 1], [1, 1, 0]]
print("m ", m)
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(wolkeSTL))
ax.scatter(x, y, z, c="red")
#axes.scatter(x1, y1, z1, c="green")
# Auto scale to the mesh size
#scale = m1.points.flatten()
#axes.auto_scale_xyz(scale, scale, scale)
