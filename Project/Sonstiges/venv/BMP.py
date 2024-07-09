import cv2
import numpy as np
import os

class BMP:
    def __init__(self, path, data):
        self.path = path     #Pfadangabe
        self.data = data     #Name der Bitmapdatei
        self.bild = os.path.join(path, data)   #Pfad und Dateiangabe

    #Wandle Bild in Numpy Array und gebe Array zurück
    def bmp_to_array(self):
        image = cv2.imread(self.bild)   #Einlesen eines Bildes und Ablage in image
        p = np.array(image)             #Wandle Bild in numpy Array p um
        return p