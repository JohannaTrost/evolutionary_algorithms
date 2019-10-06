import pybullet as p
import pybullet_data
import time
import math
from Shapes import *

##########################################
gui = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE), p.createVisualShape(p.GEOM_PLANE))




editor = UrdfEditor()

b1 = Box("B1")
b2 = Box("B2")

j1 = UrdfJoint("B1", "B2", "J1", [0,0,1.5], [0,0,0], [1,0,0], p.JOINT_SPHERICAL)


editor.addLink(b1)
editor.addLink(b2)

editor.addJoint(j1)

#id = editor.createMultiBody(physicsClientId=gui)
#p.createConstraint(id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1]) #Anchors the creature to the ground.

editor.saveUrdf("test.urdf")
p.loadURDF("test.urdf")

p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
  p.stepSimulation(physicsClientId=gui)
  time.sleep(0.01)
