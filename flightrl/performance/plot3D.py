import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    path = "/home/ysa/workspace/flightmare_unitytest/flightrl/air_planner/performance/results"
    fig = plt.figure(dpi = 150)
    ax = Axes3D(fig)
    ax.set_xlim(-6.5, 9)
    ax.set_ylim(-6, 6)
    ax.set_zlim(2.5, 5.5)

    num_env = 50
    for i in range(num_env):
        data = np.loadtxt(path+"/"+str(i)+"state.txt")

        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        ax.scatter(x, y, z, c = "g", label = "quad "+str(i))
    fig.legend()
    fig.savefig(path+"/50-3D.png")


if __name__ == "__main__":
    main()
