import pybullet as p
import pybullet_data
import time
from urdfEditor import *

boxId = p.loadURDF("test.urdf",[5,5,5], .getQuaternionFromEuler([0,0,0]))
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
