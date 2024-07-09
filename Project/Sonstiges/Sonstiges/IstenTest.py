import numpy as np
wolke = []

for i in range(10):
    wolke.append([i, 1, 2])

print(wolke)
print([spalte[1] for spalte in wolke])

arr1 = [[0, 0, 0],
        [0, 4, 0],
        [5, 0, 0],
        [5, 4, 0],
        [0, 0, 3],
        [0, 4, 3],
        [5, 0, 3],
        [5, 4, 3]]

a = np.array(arr1)
print(a[:,1])