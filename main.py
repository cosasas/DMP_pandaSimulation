import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import load_panda
import dmp_main
import config

createPath = False
useDMP = True
config.avoid_function = True
numOfSteps = 500
showGraphs = True
dmp_startingPoint = [-0.35, -0.35, 0.2] # 0 is the default starting point

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

timeStep=1./60.
p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

p._cubedim = [0.3,0.3,0.2]
p._cube_mass = 0.0884
p._visualShapeId = -1
p._cubeStartPos = [0.5,0,0.0375]
p._cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.createMultiBody(0,0)
p._colCubeId = p.createCollisionShape(p.GEOM_BOX,halfExtents=p._cubedim)
p._cubeUid = p.createMultiBody(p._cube_mass,p._colCubeId,p._visualShapeId,p._cubeStartPos,p._cubeStartOrientation)

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
    panda = load_panda.PandaSim(p,[-2,0,0])
    panda_dmp = load_panda.PandaSim(p,[0,0,0])
    robot = panda.robot()
    robot_dmp = panda_dmp.robot()
    for i in range (numOfSteps):
        panda.step()
        panda_dmp.dmp_step(dmp_p,i)   
        p.stepSimulation()
        time.sleep(timeStep)   
    
