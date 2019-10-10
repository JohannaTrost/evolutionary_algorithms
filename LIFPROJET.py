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
s1 = Sphere("S1")
b2 = Box("B2")

j1 = UrdfJoint("B1", "S1", "B1S1", [0,0,0.5], [0,0,0], [1,0,0], p.JOINT_REVOLUTE)
j2 = UrdfJoint("S1", "B2", "S1B2", [0,0,0.5], [0,0,0], [0,1,0], p.JOINT_REVOLUTE)


editor.addLink(b1)
editor.addLink(s1)
editor.addLink(b2)

editor.addJoint(j1)
editor.addJoint(j2)

editor.saveUrdf("test.urdf")
id = p.loadURDF("test.urdf")

p.setJointMotorControl2(bodyIndex=id, controlMode=p.VELOCITY_CONTROL, jointIndex=0, targetVelocity=10, force=500)

p.createConstraint(id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1]) #Anchors the creature to the ground.


p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
  p.stepSimulation(physicsClientId=gui)
  time.sleep(0.01)
