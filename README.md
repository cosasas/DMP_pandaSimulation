# DMP_pandaSimulation

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

Inside avoid folder there is also an implementation of link collision avoidance by maintaining the end effector path.

[pybullet quickstart guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#)
