# Import packages
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

X = []
Y = []
Z = []

for el in np.arange(-90, 90, 10):
    for az in np.arange(-90, 90, 10):
        X.append(el)
        Y.append(az)
        Z.append(el^2-az)

# Plot X,Y,Z
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(X, Y, Z, color='white', edgecolors='grey', alpha=0.5)  # alpha controls opacity of surface
#ax.scatter(X, Y, Z, c='red')
plt.show()
