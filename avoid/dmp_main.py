from __future__ import division, print_function
from dmp_position import PositionDMP
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import config

def DMP_main(showGraphs, dmp_startingPoint):
    # Load a demonstration file containing robot positions.
    
    demo = np.loadtxt("path2_old.dat", delimiter=" ", skiprows=1)

    tau = 0.002 * len(demo)
    t = np.arange(0, tau, 0.002)
    demo_p = demo[:, 0:3]

    N = 40  
    dmp = PositionDMP(n_bfs=N, alpha=100)
    dmp.train(demo_p, t, tau)

    if dmp_startingPoint != 0:
        dmp.p0 = dmp_startingPoint

    dmp_p, dmp_dp, dmp_ddp = dmp.rollout(t, tau)

    if showGraphs == True:
        fig1, axs = plt.subplots(3, 1, sharex=True)
        axs[0].plot(t, demo_p[:, 0], label='Demonstration')
        axs[0].plot(t, dmp_p[:, 0], label='DMP')
        axs[0].set_xlabel('t (s)')
        axs[0].set_ylabel('X (m)')

        axs[1].plot(t, demo_p[:, 1], label='Demonstration')
        axs[1].plot(t, dmp_p[:, 1], label='DMP')
        axs[1].set_xlabel('t (s)')
        axs[1].set_ylabel('Y (m)')

        axs[2].plot(t, demo_p[:, 2], label='Demonstration')
        axs[2].plot(t, dmp_p[:, 2], label='DMP')
        axs[2].set_xlabel('t (s)')
        axs[2].set_ylabel('Z (m)')
        axs[2].legend()

            # 3D plot the DMP against the original demonstration
        fig2 = plt.figure(2)
        #ax = plt.axes(projection='3d')
        ax = fig2.add_subplot(111, projection='3d')
    
        ax.plot3D(demo_p[:, 0], demo_p[:, 1], demo_p[:, 2], label='Demonstration')
        ax.plot3D(dmp_p[:, 0], dmp_p[:, 1], dmp_p[:, 2], label='DMP')
        #ax.scatter3D(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], label='Obstacles')
        ax.scatter3D(config.obstacles[:, 0], config.obstacles[:, 1], config.obstacles[:, 2], label='Obstacles')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

    return dmp_p
