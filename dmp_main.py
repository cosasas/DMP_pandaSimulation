from __future__ import division, print_function
from dmp_position import PositionDMP
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def DMP_main(showGraphs, dmp_startingPoint):
    # Load a demonstration file containing robot positions.
    
    demo = np.loadtxt("path.dat", delimiter=" ", skiprows=1)

    tau = 0.002 * len(demo)
    t = np.arange(0, tau, 0.002)
    demo_p = demo[:, 0:3]

    # TODO: In both canonical_system.py and dmp_position.py you will find some lines missing implementation.
    # Fix those first.

    N = 50  # TODO: Try changing the number of basis functions to see how it affects the output.
    dmp = PositionDMP(n_bfs=N, alpha=48.0)
    dmp.train(demo_p, t, tau)

    # TODO: Try setting a different starting point for the dmp:
    if dmp_startingPoint != 0:
        dmp.p0 = dmp_startingPoint
    # TODO: ...or a different goal point:
    #dmp.g0 = [0.2, -0.22, 0.35]

    # TODO: ...or a different time constant:
    # tau = T

    # Generate an output trajectory from the trained DMP
    dmp_p, dmp_dp, dmp_ddp = dmp.rollout(t, tau)
    #return dmp_p
    # 2D plot the DMP against the original demonstration
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
        ax = plt.axes(projection='3d')
        ax.plot3D(demo_p[:, 0], demo_p[:, 1], demo_p[:, 2], label='Demonstration')
        ax.plot3D(dmp_p[:, 0], dmp_p[:, 1], dmp_p[:, 2], label='DMP')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

    return dmp_p
