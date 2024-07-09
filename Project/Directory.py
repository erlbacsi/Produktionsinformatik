import os

class Directory:
    def __init__(self, path):
        self.path = path

    #Alle Dateinamen aus dem Ordner Path in eine Liste speichern und zurückgeben
    def data_name_list(self):
        print(self.path)
        for root, dir, files in os.walk(self.path, topdown=True):
            pfad = root
            dateien = files
        print(pfad)
        print("Anzahl Bitmaps: ", len(dateien))
        return dateien, pfad