import Cloud
import pyvista as pv
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import cv2

class Shaping:
    def __init__(self):
        self.randomP = []  #Zufallige Punkte in der Punktewolke
        self.randomT = []  #Zufällige Dreiecke mit Koordinaten (insgesamt 9 Punkte)
        self.randomN = []  #Normalenvektoren der Dreiecke
        self.pointMatrix = []  #enthält die zufälligen Punkte auf den Dreiecken
        self.uniqueNormVector = []  #Normalenvektor des aufgespannten Dreiecks
        self.uniqueMatrix = []  #enthält alle uniqueData [d1, d2, d3, alpha1, alpha2, alpha3] Listen und der erste Eintrag ist der des einzigartigen Dreiecks uas der Punktewolke
        self.plotNormals = []  #Visualisiere Normalen zum checken der Winkel


    def objectTransformation(self, dataName, wolke, uniqueData, randomTrPoints, testCases=10, numberTriangles=3):
        fehlerL = 0
        fehlerW = 0
        # Matrix welche Differenz zwischen uniqueData und uniqueMatrix enthält
        diffMatrix = []
        # Ein Datensatzt welcher Differenzen enthält
        eintrag = []
        # Lösungskandidaten Differenzen
        possibleSolution = []
        # Grenzwerte: 4 Grad und Abstandswert von 4mm
        maxDiffDegree = math.radians(4)
        maxDiffLength = 4

        stlReader = Cloud.StlReader()
        triangles, normals, maxWert, stl_points = stlReader.readMesh(dataName)

        # Berechne für STL Datei mögliche unique Dreiecke
        for i in range(testCases):
            # Zufällige Wahl der Dreiecke und Rückgabe der 3 Punkte, Normalen und Dreicksdaten
            randomTrPoint, randomNormals, randomTriangles = self.randomTrianglePoints(triangles, normals, numberTriangles)
            self.pointMatrix.append(randomTrPoint)
            # Triangles in cloud Form wandeln, um den Unique Normalenvektor korrekt ausrichten zu können
            unpackTriangles = triangles[:]
            unpacked1 = [p[0] for p in unpackTriangles]
            unpacked2 = [p[1] for p in unpackTriangles]
            unpacked3 = [p[2] for p in unpackTriangles]
            unpackedTriangles = unpacked3 + unpacked2 + unpacked1
            unique, normal = self.uniqueTriangleData(randomTrPoint, randomNormals, unpackedTriangles)
            self.uniqueMatrix.append(unique)

        # Berechne Differenzen zwischen den Dreiecksdaten
        for data in self.uniqueMatrix:
            solution = True
            # gehe alle Einträge eines Datensatzes durch und prüfe ob die Differenzen im Grenzbereich liegen
            for i in range(len(data)):
                # Ersten drei Werte beinhalten Distanz Daten
                if i >= 0 and i <= 2:
                    diff = data[i] * maxWert - uniqueData[i]    # Multiplikation mit maxWert damit ich hier die skalierten Daten auf die Proportionen der STL-Datei umrechnen kann
                    diffAbs = abs(diff)
                    eintrag.append(diffAbs)
                    if diffAbs > maxDiffLength:
                        fehlerL = fehlerL + 1
                        solution = False
                # Hinteren drei Werte beinhalten Winkel Informationen
                else:
                    diff = data[i] - uniqueData[i]
                    diffAbs = abs(diff)
                    eintrag.append(diffAbs)
                    if diffAbs > maxDiffDegree:
                        fehlerW = fehlerW + 1
                        solution = False
            diffMatrix.append(eintrag[:])
            if solution == True:
                possibleSolution.append(eintrag[:])
            eintrag.clear()
        #print("Differenzen: ", diffMatrix)
        print("Längen Fehler: ", fehlerL)
        print("Winkel Fehler: ", fehlerW)
        print("Lösungen: ", possibleSolution)
        # Beste Lösung aus den Kandidaten aussuchen -> mit geringsten Differenzen
        if len(possibleSolution) == 0:
            bestSolution = []
            finalPoints = bestSolution
            print("Keine Lösung gefunden")
            return [], [], []
        # gibt nur eine Lösungsmöglichkeit
        elif len(possibleSolution) == 1:

            index = diffMatrix.index(possibleSolution[0])
            finalPoints = self.pointMatrix[index]

        else:
            print("Start der Ranking Funktion")
            finalPoints, wolkePoints, stlPoints = self.rankingSolutions(wolke, stl_points, randomTrPoints, possibleSolution, diffMatrix)
        print("Final Point", finalPoints)
        wolkePoints = []
        wolkeCentre = self.mittelpunkt(stl_points)
        for i in range(len(stlPoints)):
            wolkePoints.append(np.subtract(np.array(stl_points[i]), wolkeCentre))
        # Finde Rotation zwischen den Punkten heraus
        rotationsmatrix, transformation = self.rotationMatching(randomTrPoints, finalPoints, visualize=True)
        return rotationsmatrix, transformation, wolkePoints, wolkeCentre


    def rotationMatching(self, originalPoints, rotatedPoints, visualize=False):
        # Matrizen mit den Punktewolken, welche in den Ursprung verschoben wurden
        centredOriginalP = []
        centredRotatedP = []
        numberPoints = len(originalPoints)
        originalCentre = self.mittelpunkt(originalPoints)
        rotatedCentre = self.mittelpunkt(rotatedPoints)

        # Punktewolken auf (0, 0, 0) schieben -> von jedem Punkt den Mittelpunkt subatrahieren
        for i in range(numberPoints):
            centredOriginalP.append(np.subtract(np.array(originalPoints[i]), originalCentre))
            centredRotatedP.append(np.subtract(np.array(rotatedPoints[i]), rotatedCentre))

        if visualize:
            a = self.createPlot()
            self.pointAdd(centredOriginalP, a)
            self.pointAdd(centredRotatedP, a)
            self.plotPoints()
            cv2.waitKey(0)

        # Werden die beiden Matrizen andersrum multipliziert wird die Rotation von RotatedP zu OriginalP berechnet
        h = np.transpose(centredRotatedP) @ np.array(centredOriginalP)
        print("h: ", h)
        u, s, vt = np.linalg.svd(h)

        # Rotation
        r = vt.T @ u.T

        # Prüfe ob eine Reflektion vorliegt -> wenn ja dann 3 Spalte mit -1 multiplizieren
        if np.linalg.det(r) < 0:
            vt[2, :] *= -1
            r = vt.T @ u.T

        # Transformation
        #print("point original", originalCentre)
        t = -r @ np.array(rotatedCentre).reshape(-1, 1) + np.array(originalCentre).reshape(-1, 1)
        #print("Translation: ", t)

        # Drehe nun Ausgangspunkte auf die gedrehte Zielposition
        rotatedA = r @ np.transpose(centredRotatedP)

        if visualize:
            a = self.createPlot()
            self.pointAdd(rotatedA.T, a)
            self.pointAdd(centredOriginalP, a)
            self.plotPoints()
        return r, t


    # Funktionen zum Plotten von Punkten und verbinden der entpsrechenden Punkte mit Linien
    def createPlot(self):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection="3d")
        return ax


    # Alle eingegebenen Punkte werden gezeichnet und mit einer Linie in ihrer Einfügereihenfolge verbunden
    def pointAdd(self, points, ax):
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


    def plotPoints(self):
        plt.show()


    # Lösungskandidaten filtern und den besten Auswählen
    def rankingSolutions(self, wolke, stlPoints, randomTrPoints, possibleSolution, diffMatrix):
        possiblePointList = []
        # Differenzen der Punkte einer rotierten Punktewolke zur STL Datei
        diffValues = []
        # Summen der Differenzen für alle möglichen Lösungen
        diffValuesCollect = []
        stlPointsCentred = []
        wolkePointsCentred = []
        anzahl = len(possibleSolution)

        # Alle Punkte der uniquen Lösungsdreiecke ablegen
        for i in range(anzahl):
            ind = diffMatrix.index(possibleSolution[i])
            pkt = self.pointMatrix[ind]
            possiblePointList.append(pkt)

        # STL Datei Punkte in Ursprung verschoben
        stlCentre = self.mittelpunkt(stlPoints)
        for i in range(len(stlPoints)):
            stlPointsCentred.append(np.subtract(np.array(stlPoints[i]), stlCentre))

        # Punktewolke in Ursprung verschoben
        wolkeCentre = self.mittelpunkt(wolke)
        for i in range(len(wolke)):
            wolkePointsCentred.append(np.subtract(np.array(wolke[i]), wolkeCentre))

        # Suche für jede Lösung die Rotation und berechne den Offset zwischen der STL und der gedrehten Punktewolke um die beste Lödung raus zu filtern
        for i in range(anzahl):
            rotationsmatrix, transformation = self.rotationMatching(randomTrPoints, possiblePointList[i], visualize=False)
            wolkeRotated = rotationsmatrix @ np.transpose(wolkePointsCentred)
            wolkeR = wolkeRotated.T
            for j in range(len(wolkeR)):
                minValue = float("inf")
                point = wolkeR[j]
                # Berechne für jeden Punkt in Punktewolke nähesten Nachbarn aus der Punktewolke
                for k in range(len(stlPoints)):
                    vektor = np.subtract((np.array(point)), (np.array(stlPointsCentred[k])))
                    dist = math.sqrt(sum([x ** 2 for x in vektor]))
                    if dist < minValue:
                        minValue = dist
                diffValues.append(minValue)
            diffValuesCollect.append(sum(diffValues))
            diffValues.clear()

        minDiffIndex = diffValuesCollect.index(min(diffValuesCollect))
        index = diffMatrix.index(possibleSolution[minDiffIndex])
        #print("unique Solution: ", self.uniqueMatrix[index])
        finalPoints = self.pointMatrix[index]

        return finalPoints, wolkePointsCentred, stlPointsCentred


    # Zufällige Wahl von drei Dreiecken in der STL Datei
    def randomTrianglePoints(self, triangles, normals, numberTriangles):
        points = []
        randomNormals = []
        possibleTriangles = []
        triangleCount = len(triangles)
        for i in range(numberTriangles):
            randomNumber = random.randint(0, triangleCount - 1)
            triangle = triangles[randomNumber]

            #Stelle sicher dass die ersten beiden Punkte nicht auf einer Ebene liegen
            if possibleTriangles.__contains__(triangle):
                while True:
                    randomNumber = random.randint(0, triangleCount - 1)
                    triangle = triangles[randomNumber]
                    #wenn verschieden dann einfach mit letzer Dreieckswahl weitermachen
                    if not possibleTriangles.__contains__(triangle):
                        break
            possibleTriangles.append(triangle)
            punkt = self.random_triangle_point(possibleTriangles[i])
            points.append(punkt)
            randomNormals.append(normals[randomNumber])
        return points, randomNormals, possibleTriangles


    def objectEstimation(self, cloud, numberTriangles=3, downsample=3, showNormals=False):
        #Zufälliger Punkt auf dem jeweiligen Dreieck
        randomTrPoint = []
        #plotCloud enthält alle Punkte die gezeichnet werden sollen und kann erweitert werden (cloud soll nicht erweitert werden)
        plotCloud = cloud[:]
        #enthält die Daten für das plotten des unique Dreieck -> erste Zahl 4 notwendig für 4 Folgepunkte, damit Polydata Objekt Punkte verbindet
        newTriangle = [4,]
        self.randomPoints(cloud, numberTriangles)
        triangles, indizes, normals = self.randomTriangulation(cloud, downsample)
        normals = self.update_normals(cloud, normals, triangles)
        #Bestimme für jedes Dreieck einen zufälligen Punkt auf diesem
        for i in range(numberTriangles):
            #zufälliger Punkt auf Dreieck
            punkt = self.random_triangle_point(triangles[i])
            randomTrPoint.append(punkt)
            #Alte Punktewolke wurde kopiert und wird für das Plotten erweitert
            plotCloud.append(punkt)
            index = plotCloud.index(punkt)
            newTriangle.append(index)
        self.pointMatrix.append(randomTrPoint)
        #ersten Index nochmal an den Schluss damit Dreieck geschlossen wird
        newTriangle.append(newTriangle[1])
        #Dreieck mit den Indizes in die Liste anfügen damit es geplottet werden kann
        indizes.append(newTriangle)
        #Distanzen zwischen Punkten des randomTrPoint und Winkel zwischen Normalen
        uniqueData, uniqueNormal = self.uniqueTriangleData(randomTrPoint, normals, cloud)
        self.uniqueNormVector = uniqueNormal
        self.uniqueMatrix.append(uniqueData)

        # Wenn True dann zeige Grafik mit Normalenvektoren
        if showNormals:
            for i in range(len(normals)):
                self.plotNormals.append(list(normals[i]))
            zentrum = [0, 0, 0]
            self.plotNormals.append(list(uniqueNormal))
            self.plotNormals.append(zentrum)
            plotNormalsIndex = []
            for i in range(len(normals)):
                plotNormalsIndex.append(2)
                plotNormalsIndex.append(self.plotNormals.index(zentrum))
                plotNormalsIndex.append(self.plotNormals.index(self.plotNormals[i]))
            plotNormalsIndex.append(2)
            plotNormalsIndex.append(self.plotNormals.index(zentrum))
            plotNormalsIndex.append(self.plotNormals.index(list(uniqueNormal)))
            self.plot_edges(self.plotNormals, plotNormalsIndex, True)
        return plotCloud, triangles, indizes, normals, uniqueData, randomTrPoint


    #Berechne Distanzen zwischen Punkten und die Winkel zwischen den Normalenvektoren
    def uniqueTriangleData(self, randomTrPoint, normals, cloud):
        uniqueData = []
        mittelpunkt = self.mittelpunkt(cloud)

        v1 = np.subtract((np.array(randomTrPoint[1])), (np.array(randomTrPoint[0])))
        d1 = math.sqrt(sum([x ** 2 for x in v1]))
        v2 = np.subtract((np.array(randomTrPoint[2])), (np.array(randomTrPoint[0])))
        d2 = math.sqrt(sum([x ** 2 for x in v2]))
        v3 = np.subtract((np.array(randomTrPoint[2])), (np.array(randomTrPoint[1])))
        d3 = math.sqrt(sum([x ** 2 for x in v3]))

        uniqueData.append(d1)
        uniqueData.append(d2)
        uniqueData.append(d3)
        #Normalenvektor des uniquen Dreiecks
        normalUniqueVektor = list(self.norm(np.cross(v1, v2)))
        normalUniqueVektor = self.updateNormal(randomTrPoint, normalUniqueVektor, mittelpunkt)
        self.angleDegrees(normals, normalUniqueVektor, uniqueData)
        return uniqueData, normalUniqueVektor


    #Berechne Winkel zwischen Normalenvektoren der Dreiecke und dem Normalenvektor des Unique Dreieck
    def angleDegrees(self, normals, uniqueNormal, uniqueData):
        uniqueBetrag = math.sqrt(sum([x ** 2 for x in uniqueNormal]))
        for i in range(len(normals)):
            dotProduct = np.dot(normals[i], uniqueNormal)
            if dotProduct == 1:
                print("Winkel 0")
            normalBetrag = math.sqrt(sum([x ** 2 for x in normals[i]]))
            wert = dotProduct / (uniqueBetrag * normalBetrag)
            if wert <= 1 and wert >= -1:
                winkel = math.acos(wert)  # math.degrees() Umrechnung ins Gradmaß
            else:
                winkel = float("inf")
            uniqueData.append(winkel)


    #Suche einen zufälligen Punkt auf dem Dreieck aus, mit Hilfe einer Linearkombination auf Basis der aufgespannten Ebene
    def random_triangle_point(self, triangleKoord):
        triangleKoordNp = np.array(triangleKoord)
        point_matrix = np.transpose(triangleKoordNp.reshape((3, 3)))
        r1 = random.random()
        r2 = random.random()
        random_point = ((1 - np.sqrt(r1)) * point_matrix[:, 0] + (np.sqrt(r1) * (1 - r2)) * point_matrix[:, 1] + (np.sqrt(r1) * r2) * point_matrix[:, 2])
        return list(random_point)


    # Suche drei zufällige Punkte aus der Wolke heraus bei denen ein Punkt einen unterschiedlichen x-Wert besitzt (Zeile) und einer einen unterschiedlichen y-Wert (Spalte)
    # So wird sichergestellt dass kein Dreieck auf einer Fläche des Objektes gebildet wird
    def randomPoints(self, cloud, numberTriangles):
        # Zuerst letzte Ebene rausfiltern damit keine Punkte davon gewählt werden
        # Denn für die Dreiecksbildung wird immer das Element der Ebene oben drüber verwendet -> wenn Punkt aus letzter Ebene verwendet wird funktioniert das Verfahren nicht
        number = len(cloud)
        zLastEbene = cloud[number - 1]
        lastEbene = [p for p in cloud if p[2] == zLastEbene[2]]
        numberNeu = number - len(lastEbene) - 1
        for i in range(numberTriangles):
            randIndex = random.randint(0, numberNeu)
            punkt = cloud[randIndex]
            # am Ende steht hier x-Wert von Punkt 3
            x = punkt[0]
            y = punkt[1]
            # Am Ende prüfen ob alle Punkte auf einer Fläche liegen -> wenn ja dann einen Punkt auf einer anderen Zeile wählen
            # Prüfe ob alle Punkte gleichen y-Wert haben -> wenn ja dann suche einen mit anderem Spaltenwert
            if self.randomP.__contains__(punkt) or (i == 2 and self.randomP[0][0] == self.randomP[1][0] and self.randomP[0][0] == x) or (i == 2 and self.randomP[0][1] == self.randomP[1][1] and self.randomP[0][1] == y):
                #xNeu = x
                while True:
                    indexNeu = random.randint(0, numberNeu)
                    punkt = cloud[indexNeu]
                    xNeu = punkt[0]
                    yNeu = punkt[1]
                    if not (xNeu == x) and not (yNeu == y) and not(self.randomP.__contains__(punkt)) and not ((self.randomP[0][0] == self.randomP[1][0] and self.randomP[0][0] == xNeu) or (self.randomP[0][1] == self.randomP[1][1] and self.randomP[0][1] == yNeu)):
                        break
            self.randomP.append(punkt)


    def randomTriangulation(self, cloud, downsample):
        triangles = []
        triangle_index = []
        normals = []
        for i in range(len(self.randomP)):
            edges = self.randomShape2d(cloud, self.randomP[i], downsample)
            ebene = self.randomP[i][2]
            ebene += 1
            nextArea = [p for p in cloud if p[2] == ebene]
            # Suche nächste Ebene
            while len(nextArea) == 0:
                ebene += 1
                nextArea = [p for p in cloud if p[2] == ebene]
                if ebene > cloud[len(cloud) - 1][2]:
                    break

            # alle Kanten durchgehen und Dreieck mit Ebene darüber bilden
            if edges == []:
                continue
            p1 = edges[0]  # Punkte p1 und p2 der betrachteten Kante
            p2 = edges[1]
            pAbove1 = self.nearest_above(p1, p2, nextArea)  # Zwei Dreiecke modellieren die ein Quadrat bilden
            triangles, triangle_index, normals = self.randomTriangleNormals(cloud, p1, p2, pAbove1, triangles, triangle_index, normals)
        return triangles, triangle_index, normals


    def randomShape2d(self, cloud, punkt, downsample):
        zEbene = punkt[2]
        edges = []
        area = [p for p in cloud if p[2] == zEbene]
        size = len(area)

        startPoint = punkt
        lineX = startPoint[0]
        sameLine = [p for p in area if p[0] == lineX]  # alle Punkte auf selber Zeile
        nextX = lineX + 1
        nextLine, nextnextX = self.nextLineOut(area, nextX, size)  # alle Punkte auf nächster Zeile
        nextnextX += 1
        nextnextLine, n = self.nextLineOut(area, nextnextX, size)  # Auf Vertikalen parallelen Strecken kann durch downsample unregelmäßiges Gitter entstehen
        # startIndex = i + 1        #der Index des nächsten Punktes der Zeile
        min = float('inf')
        nextYpoint = []

        for j in range(len(sameLine)):  # suche nähesten Partner in gleicher Zeile
            dist = np.subtract(sameLine[j], startPoint)
            dist = np.absolute(dist)
            laenge = math.sqrt(sum([x ** 2 for x in dist]))
            if laenge == 0:  # Duplikate eliminieren
                continue
            if laenge < min:
                min = laenge
                if min <= downsample + 1:  # Wenn min kleiner oder gleich dem regulären Abstand ist. Wenn größer dann ist Punkt zu weit weg
                    nextYpoint = sameLine[j]
        # wenn kein Nachbar auf gleicher Zeile vorhanden suche nach einem in nächster Zeile
        nextXpoint = []
        if len(nextYpoint) == 0:
            min = float('inf')
            for k in range(len(nextLine)):  # Prüfe ob es kürzeste Verbdinung zu einem Punkt in nächster Zeile gibt
                dist = np.subtract(nextLine[k], startPoint)
                laenge = math.sqrt(sum([x ** 2 for x in dist]))
                if laenge < min:
                    min = laenge
                    if min <= downsample + 1:  # Wenn min kleiner oder gleich dem regulären Abstand ist. Wenn größer dann ist Punkt zu weit weg
                        nextXpoint = nextLine[k]
            if len(nextXpoint) == 0:
                for k in range(len(nextnextLine)):
                    dist = np.subtract(nextnextLine[k], startPoint)
                    laenge = math.sqrt(sum([x ** 2 for x in dist]))
                    if laenge < min:
                        min = laenge
                        if min <= downsample + 1:  # Wenn min kleiner oder gleich dem regulären Abstand ist. Wenn größer dann ist Punkt zu weit weg
                            nextXpoint = nextnextLine[k]

        if not (len(nextYpoint) == 0):
            edges.append(startPoint)
            edges.append(nextYpoint)
        if not (len(nextXpoint) == 0):
            edges.append(startPoint)
            edges.append(nextXpoint)
        return edges


    def randomTriangleNormals(self, cloud, p1, p2, pAbove1, triangles, triangle_index, normals):
        vector1 = np.subtract(p2, p1)  # Erstes Dreieck
        vector2 = np.subtract(pAbove1, p1)
        triangles.append([p1, p2, pAbove1])
        triangle_index.append([4, cloud.index(p1), cloud.index(p2), cloud.index(pAbove1), cloud.index(p1)])
        normals.append(self.norm(np.cross(vector1, vector2)))
        return triangles, triangle_index, normals


    def nextLineOut(self, area, nextX, size):
        nextLine = [p for p in area if p[0] == nextX]  # alle Punkte auf nächster Zeile
        while len(nextLine) == 0:  # Suche nächsten Zeilenindex der Werte enthält
            nextX = nextX + 1
            nextLine = [p for p in area if p[0] == nextX]
            if nextX > area[size - 1][0]:
                break
        return nextLine, nextX


    def nearest_above(self, p1, p2, nextArea):
        minLaenge = float('inf')
        pTop = 0
        for b in range(len(nextArea)):
            pAbove = nextArea[b]  # Suche kleinstes Dreieck
            vektor1 = np.subtract(pAbove, p1)
            vektor2 = np.subtract(pAbove, p2)
            if pAbove == p1 or pAbove == p2:  # schaue dass keine bereits verwendeten Punkte genommen werden
                continue
            laenge1 = math.sqrt(sum([x ** 2 for x in vektor1]))
            laenge2 = math.sqrt(sum([x ** 2 for x in vektor2]))
            lgesamt = laenge1 + laenge2
            if minLaenge > lgesamt:
                minLaenge = lgesamt
                pTop = pAbove
        return pTop


    def plot_edges(self, points, lines, normal=False):
        line = np.hstack(lines)
        pdata = pv.PolyData(np.array(points))
        pdata.lines = line
        pl = pv.Plotter()
        pl.add_mesh(pdata)
        # Normalenvektor des unique Dreieck visualisieren
        if normal:
            size = len(points)
            p = np.array([points[size - 1], points[size - 2]])
            pl.add_lines(p, color="red")
        pl.show()


    def mittelpunkt(self, cloud):
        xValues = [p[0] for p in cloud]
        yValues = [p[1] for p in cloud]
        zValues = [p[2] for p in cloud]
        xAverage = sum(xValues) / len(xValues)
        yAverage = sum(yValues) / len(yValues)
        zAverage = sum(zValues) / len(zValues)
        return [xAverage, yAverage, zAverage]


    # TODO: mittelpunkt nicht innerhalb -> Sonderfall
    def update_normals(self, cloud, normals, triangles):
        center = self.mittelpunkt(cloud)
        for i in range(len(normals)):
            normal = self.norm(normals[i])
            midpoint = np.subtract(center, self.mittelpunkt(triangles[i]))  # vektor von mittelpunkt zu mitte Dreieck
            mNormal = self.norm(midpoint)
            product = np.dot(normal, mNormal)  # Skalarprodukt bei dem somit cos(theta) entscheidet über Vorzeichen -> weiß ob Winkel größer oder kleiner 90 Grad ist
            if product > 0:  # Normale zeigt in Mitte des Bauteils
                normals[i] *= -1
        return normals


    # Richtung des Normalenvektors eines unique Dreieck anpassen
    # soll einheitlich nach außen zeigen
    def updateNormal(self, points, normal, midpoint):
        normal = self.norm(np.array(normal))
        midpointVector = np.subtract(midpoint, self.mittelpunkt(points))  # vektor von mittelpunkt zu mitte Dreieck
        mNormal = self.norm(midpointVector)
        product = np.dot(normal, mNormal)  # Skalarprodukt bei dem somit cos(theta) entscheidet über Vorzeichen -> weiß ob Winkel größer oder kleiner 90 Grad ist
        if product > 0:  # Normale zeigt in Mitte des Bauteils
            normal *= -1
        return normal


    def norm(self, vektor):
        betrag = np.sqrt(np.sum(vektor ** 2))
        normV = vektor / betrag
        return normV
