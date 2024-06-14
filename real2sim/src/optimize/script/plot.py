import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

for mode in ["onestep", "boostrap"]:
    data = np.loadtxt("/home/ysa/workspace/flightmare/flightmare/real2sim/src/optimize/result/"+mode+".csv", delimiter=',', dtype=float)

    fig = plt.figure(dpi = 150)
    ax = Axes3D(fig)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-2.5, 2.5)
    ax.set_zlim(0.5, 1.5)

    x_gt = data[:, 0]
    y_gt = data[:, 1]
    z_gt = data[:, 2]

    x = data[:, 3]
    y = data[:, 4]
    z = data[:, 5]

    ax.scatter(x_gt, y_gt, z_gt, c = "b", label = "gt")
    ax.scatter(x, y, z, c = "g", label = mode)
    fig.legend()
    fig.savefig("/home/ysa/workspace/flightmare/flightmare/real2sim/src/optimize/result/"+mode+".png")
    fig = plt.figure(dpi = 150)
    plt.scatter(x_gt, y_gt, c = "b", label = "gt")
    plt.scatter(x, y, c = "g", label = mode)
    plt.xlim(-1, 1)
    plt.ylim(-2.5, 2.5)
    plt.legend()
    plt.savefig("/home/ysa/workspace/flightmare/flightmare/real2sim/src/optimize/result/"+mode+"2D.png")
print("done")
