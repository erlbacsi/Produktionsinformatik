'''
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-------------------------Hier Funktionen um die Außenseiten eines Objektes vollständig zu Triangulieren------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
'''
'''
    Berechne hier die Form einer Ebene auf Basis des Abstandes zwischen zwei Punkten (=downsample)
    #Somit gebe Liste zurück die immmer zwei Punkte enthält welche mit einem Punkt der Ebene darüber dann ein Triangle bilden sollen
    #Rückgabe ist somit eine Liste der Triangles einer Ebene, welche jeweils zwei Punkte eines Triangles enthält
    def shape_2D(self, cloud, zEbene, downsample=3):
        edges = []
        indizes = []
        area = [p for p in cloud if p[2] == zEbene]
        size = len(area)
        print("CLoud", area)
        for i in range(size):
            startPoint = area[i]
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
                laenge = math.sqrt(sum([x ** 2 for x in dist]))
                if laenge == 0 or dist[0] < 0 or dist[1] < 0:  # Duplikate eliminieren
                    continue
                if laenge < min:
                    min = laenge
                    if min <= downsample + 1:  # Wenn min kleiner oder gleich dem regulären Abstand ist. Wenn größer dann ist Punkt zu weit weg
                        nextYpoint = sameLine[j]
                        index = j
            min = float('inf')
            nextXpoint = []
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
                edges.append([startPoint, nextYpoint])
                indizes.append([2, cloud.index(startPoint), cloud.index(nextYpoint)])
            if not (len(nextXpoint) == 0):
                edges.append([startPoint, nextXpoint])
                indizes.append([2, cloud.index(startPoint), cloud.index(nextXpoint)])
        return edges, indizes

    #Trianguliere Flächen zwischen zwei Punkteebnenen
    #Gebe die Anzahl der Bilder (=Ebnene) an und den Abstand zwischen zwei benachbarten Punkten in einer Ebene (=downsample Size)
    def triangulation(self, cloud, anzahlBilder, downsample):
        ebene = 0
        triangles = []
        triangle_index = []
        normals = []

        for i in range(anzahlBilder - 1):
            edges, indizes = self.shape_2D(cloud, i, downsample)
            print("Edges", edges)
            ebene += 1
            nextArea = [p for p in cloud if p[2] == ebene]
            while len(nextArea) == 0:  # Suche nächste Ebene
                ebene += 1
                nextArea = [p for p in cloud if p[2] == ebene]
                if ebene > cloud[len(cloud) - 1][2]:
                    break

            for a in range(len(edges)):  # alle Kanten durchgehen und Dreieck mit Ebene darüber bilden
                p1 = edges[a][0]  # Punkte p1 und p2 der betrachteten Kante
                p2 = edges[a][1]

                pAbove1 = self.nearest_above(p1, p2, nextArea)  # Zwei Dreiecke modellieren die ein Quadrat bilden
                pAbove2 = self.nearest_above(p2, pAbove1, nextArea)
                if a == 0:
                    print("above1", pAbove1)
                    print("above2", pAbove2)
                triangles, triangle_index, normals = self.triangles_normals(cloud, p1, p2, pAbove1, pAbove2, triangles,
                                                                       triangle_index, normals)
        print("triangleIndex", triangle_index)
        return triangles, triangle_index, normals

    def triangles_normals(self, cloud, p1, p2, pAbove1, pAbove2, triangles, triangle_index, normals):
        vector1 = np.subtract(p2, p1)  # Erstes Dreieck
        vector2 = np.subtract(pAbove1, p1)
        triangles.append([p1, p2, pAbove1])
        triangle_index.append([4, cloud.index(p1), cloud.index(p2), cloud.index(pAbove1), cloud.index(p1)])
        normals.append(np.cross(vector1, vector2))

        vector3 = np.subtract(pAbove2, p2)  # Zweites Dreieck
        vector4 = np.subtract(pAbove2, pAbove1)
        triangles.append([p2, pAbove1, pAbove2])
        triangle_index.append([4, cloud.index(p2), cloud.index(pAbove1), cloud.index(pAbove2), cloud.index(p2)])
        normals.append(np.cross(vector3, vector4))
        return triangles, triangle_index, normals
'''