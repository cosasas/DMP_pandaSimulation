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

config.obstacles = 0.1* np.random.random((num_obstacles_points, 3))

#configure space to avoid
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

            R_dy = np.array([[             1,                0,               0],#einai to RX
                             [             0,   np.cos(phi_dy), -np.sin(phi_dy)],
                             [             0,   np.sin(phi_dy), np.cos(phi_dy)]])
                             
            obj_vec = obstacle - y
            obj_vec = np.multiply(R_dy,obj_vec)
            phi = np.arctan2(obj_vec[2], obj_vec[1])
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




