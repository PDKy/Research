import numpy as np

a = np.array([1,2,3,4,5])
b = np.array([6,7,8,9,10])
c = np.hstack([a,b])
print(c[:5])
print(c[-5:])
print(c[:5] @ c[-5:])