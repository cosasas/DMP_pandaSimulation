# -*- coding: utf-8 -*-
"""
Created on Sat Apr 25 15:35:44 2020

@author: nick_
"""
import config
import numpy as np 
import matplotlib.pyplot as plt

beta = 20.0 / np.pi
gamma = 120

R_halfpi = np.array(
    [   [1,                  0,                    0],
        [0,np.cos(np.pi / 2.0), -np.sin(np.pi / 2.0)],
        [0,np.sin(np.pi / 2.0),  np.cos(np.pi / 2.0)],
    ]
)
num_obstacles_points = 100
#config.obstacles = np.random.random((num_obstacles, 3)) * 2 - 1
#config.obstacles = np.random.random((num_obstacles, 3)) /4 +0.4
#config.obstacles = np.random.random((num_obstacles, 3))/5 + 0.4
#config.obstacles[:,1] = config.obstacles[:,1] - 0.2
#config.obstacles[:,0] = config.obstacles[:,0] + 0.05
#obstacles = config.obstacles
config.obstacles = 0.1* np.random.random((num_obstacles_points, 3))
#config.obstacles = 0.3* np.random.random((num_obstacles_points, 3))+0.5
#config.obstacles[:,1] = config.obstacles[:,1]-0.5                              #configure space to avoid
#config.obstacles[:,2] = config.obstacles[:,2]-0.5
config.obstacles[:,0] = config.obstacles[:,0]-0.25
config.obstacles[:,1] = config.obstacles[:,1]-0.25                            
config.obstacles[:,2] = config.obstacles[:,2]+0.45

obstacles = config.obstacles

def avoid_obstacles(y, dy, goal):
    p = np.zeros(3)
    #print(dy)
    for obstacle in obstacles:
        # based on (Hoffmann, 2009)
 
        # if we're moving
        if np.linalg.norm(dy) > 1e-5:
 
            # get the angle we're heading in
            phi_dy = -np.arctan2(dy[1], dy[0]) 
            #print(phi_dy,"\n")
            R_dy = np.array([[             1,                0,               0],#einai to RX
                             [             0,   np.cos(phi_dy), -np.sin(phi_dy)],
                             [             0,   np.sin(phi_dy), np.cos(phi_dy)]])
                             
#            R_dy = np.array([[ np.cos(phi_dy),                0,  np.sin(phi_dy)],
#                             [              0,                1,               0],
#                             [-np.sin(phi_dy),                0,  np.cos(phi_dy)]])
            
#            R_dz = np.array([[ np.cos(phi_dy),  -np.sin(phi_dy),               0],
#                             [ np.sin(phi_dy),   np.cos(phi_dy),               0],
#                             [              0,                0,               1]])
            
            #R_dy = R_dz * R_dy * R_dx #rot_matrix
            #print(R_dy)
            # calculate vector to object relative to body
            #print(y)
            #print(obstacle)
            obj_vec = obstacle - y
            #print(obj_vec)
            # rotate it by the direction we're going 
           # obj_vec = np.dot(R_dy, obj_vec)
            obj_vec = np.multiply(R_dy,obj_vec)
            #print(obj_vec)
            # calculate the angle of obj relative to the direction we're going
            phi = np.arctan2(obj_vec[2], obj_vec[1])
            #print(phi)
            dphi = gamma * phi * np.exp(-beta * abs(phi))
            R = np.dot(R_halfpi, np.outer(obstacle - y, dy))
            pval = -np.nan_to_num(np.dot(R, dy) * dphi)
 
            # check to see if the distance to the obstacle is further than 
            # the distance to the target, if it is, ignore the obstacle
            if np.linalg.norm(obj_vec) > np.linalg.norm(goal - y):
                pval = 0
 
            p += pval
    #print(p)
    return p
#Dp = np.identity(3)
#p0 = np.zeros(3)
#gp = np.zeros(3)
#avoid_obstacles(p0, Dp, gp)
#avoid_obstacles(0, 0, 1)

# fig3 = plt.figure(3)
# ax = plt.axes(projection='3d')

# #for obstacle in obstacles:
# #        (plot_obs,) = plt.plot(obstacle[0], obstacle[1], "rx", mew=3)
# ax.scatter3D(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], label='Obstacles')
# #print(plot_obs)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()
# plt.show()




