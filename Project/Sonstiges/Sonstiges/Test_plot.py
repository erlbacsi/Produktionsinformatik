import matplotlib.pyplot as plt
import numpy as np

normals = []
p1 = [550, 501, 1]
p2 = [550, 500, 1]
pAbove1 = [550, 500, 2]
vector1 = np.subtract(np.array(p2) / 10, np.array(p1) / 10)  # Erstes Dreieck
print(vector1)
vector2 = np.subtract(np.array(pAbove1) / 10, np.array(p1) / 10)
print(vector2)
normals.append(np.cross(vector1, vector2))
print("punktewolke normalen ", normals)

values = [2, 5, 22]
names = ["a", "b", "c"]

plt.figure(figsize=(9, 3))

plt.subplot(1, 3, 1)
plt.scatter(names, values)
plt.subplot(1, 3, 2)
plt.bar(names, values)
plt.subplot(1, 3, 3)
plt.plot(names, values)
#plt.axis([0, 6, 0, 20])
plt.show()