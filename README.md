# DMP_pandaSimulation

This project is an implementation of trajectory planning with DMP. The software is developed in PyBullet simulation using Frank Emika Panda robot (7DoF). We are also able to deviate from the initial path to avoid obstacles.

![](/Images/avoid_obs_dmp.png)

In addition, because our robot is 7Dof we are able to implement a link collision avoidance by maintaining the end effector path. The code can be found inside avoid folder.

Path without obastacle
![](/Images/original_path.png)

Link path with obastacle, blue cube.
![](/Images/avoid2_other_view.png)


to install PyBullet use
```
pip install pybullet
```

To create path use
```
createPath = True
useDMP = False
```
If you want to change the path alter from **load_panda.py** line 63

To use DMP
```
createPath = False
useDMP = True
```

[pybullet quickstart guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#)
