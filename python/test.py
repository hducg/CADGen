import numpy as np

x = [0, 1, 2]

np.savetxt('test.txt', x, fmt='%u')
y = np.loadtxt('test.txt')
print(y)