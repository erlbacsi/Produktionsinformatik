from PIL import Image
import numpy as np
import cv2
import Cloud
import matplotlib.pyplot as plt


def bmp_read(bild):
    # im = Image.open("Teil.bmp")
    im = cv2.imread(bild)
    p = np.array(im)
    print(im.shape)

    # Skalierung der Größe für Anzeige
    scale = 30
    width = int(im.shape[1] * scale / 100)
    height = int(im.shape[0] * scale / 100)
    dim = (width, height)
    r_im = cv2.resize(im, dim, interpolation=cv2.INTER_AREA)
    print(r_im.shape)

    # width, height = im.size
    # print(width, height)

    cv2.imshow("bmp", r_im)
    cv2.waitKey()
    cv2.destroyAllWindows()
    return p

def punktewolke(arr):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection="3d")
        height = arr.shape[0]
        width = arr.shape[1]
        ax.axis([0, width, 0, height])
        #ax.set_zlim(0,100)
        print(height)

        for h in range(height):
            for w in range(width):
                x = arr[h][w][0]
                y = arr[h][w][1]
                z = arr[h][w][2]
                if x == 255 and y == 255 and z == 255:
                    ax.scatter(h, w, 1, s=0.05, color=[0,0,0])

        plt.show()

arr = bmp_read("Teil.bmp")
punktewolke(arr)


