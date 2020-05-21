import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import load_panda
import dmp_main
import config
import link_collision_avoid

createPath = False
useDMP = True
config.avoid_function = False
numOfSteps = 500
showGraphs = False
dmp_startingPoint = 0#[-0.35, -0.35, 0.2] # 0 is the default starting point

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

timeStep=1./60.
p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

p._cubedim = [0.05, 0.05, 0.05]

#p._cube_mass = 0.0884 to put gravity to object
p._cube_mass = 0
p._visualShapeId = -1
#p._cubeStartPos = [0.32,0.01,0.2]    # before avoid
p._cubeStartPos = [-0.32,0.01,0.5]   
p._cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.createMultiBody(0,0)
p._colCubeId = p.createCollisionShape(p.GEOM_BOX,halfExtents=p._cubedim)
p._cubeUid = p.createMultiBody(p._cube_mass,p._colCubeId,p._visualShapeId,p._cubeStartPos,p._cubeStartOrientation)

obstacle_position =(-0.32,0.01,0.5) 
obstacle_radius = 0.06     
                                           # we use it for threshold distance

#while (1):
if createPath == True and useDMP == False:
    panda = load_panda.PandaSim(p,[0,0,0]) # position control offset
    robot = panda.robot()
    f = open ("path.dat" , "w")
    for i in range (numOfSteps):
        pos, orn = panda.step()
        p.stepSimulation()
        time.sleep(timeStep)
        pos = [ '%.5f' % elem for elem in pos ]
        euler = str(pos)[1:-1] + ", " + str(p.getEulerFromQuaternion(orn))[1:-1]
        euler = euler.replace("'",'')
        f.write(euler.replace(',','') + "\n")
    f.close()
elif createPath == False and useDMP == True:
    dmp_p = dmp_main.DMP_main(showGraphs, dmp_startingPoint)
    #panda = load_panda.PandaSim(p,[-2,0,0])                # for second panda
    panda_dmp = load_panda.PandaSim(p,[0,0,0])
    #robot = panda.robot()
    robot_dmp = panda_dmp.robot()
    for i in range (numOfSteps):
        #panda_dmp.step()
        u_returned,flag = link_collision_avoid.link(robot_dmp, p, obstacle_position, obstacle_radius)

        panda_dmp.dmp_step(dmp_p, i, u_returned, flag)   
        p.stepSimulation()
        time.sleep(timeStep)   

