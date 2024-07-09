import numpy as np
import random
import matplotlib.pyplot as plt

#-------Plotten von Punkten und hinzufügen von Punkten--------
#Plotten erst am Ende wenn alle Punkte hinzugefügt wurden
def createPlot():
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")
    return ax

def pointPlot(points, ax):
    x = []
    y = []
    z = []
    for p in points:
        ax.scatter(p[0], p[1], p[2])
        x.append(p[0])
        y.append(p[1])
        z.append(p[2])
    x.append(x[0])
    y.append(y[0])
    z.append(z[0])
    #Punkte müsen unbedingt in Liste liegen und Werte durch Komma getrennt werden -> beim Slicen der Spalten ist das nicht der Fall
    ax.plot(x, y, z)
    plt.show()

def addPoint(p, ax):
    ax.scatter(p[0], p[1], p[2])
#--------------------------------------------------------------


def random_triangle_point(koordinaten):
    point_matrix = np.transpose(koordinaten.reshape((3,3)))
    #print(point_matrix)
    r1 = random.random()
    #print("r1 ist gleich:", r1)
    r2 = random.random()
    #print("r2 ist gleich:", r2)
    random_point = ((1-np.sqrt(r1))*point_matrix[:, 0]+(np.sqrt(r1)*(1-r2))*point_matrix[:, 1]+(np.sqrt(r1)*r2)*point_matrix[:, 2])
    #print(point_matrix[:, 0])
    #print("random point auf der Dreiecksfläche ist gleich:", random_point)
    return(random_point)

#Dreieckdaten bestehend aus drei Punkten
p = np.array([[3, 6, 0], [2, 1, 0], [8, 9, 0]])

ax = createPlot()
for i in range(300):
    x = random_triangle_point(p)
    addPoint(x, ax)

pointPlot(p, ax)


