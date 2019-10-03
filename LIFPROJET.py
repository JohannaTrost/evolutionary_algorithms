import pybullet as p
import time
from urdfEditor import *

##########################################
org2 = p.connect(p.DIRECT)
org = p.connect(p.SHARED_MEMORY)
if (org < 0):
  org = p.connect(p.DIRECT)

gui = p.connect(p.GUI)

p.resetSimulation(physicsClientId=org)



editor = UrdfEditor()

l1 = UrdfLink("L1", [UrdfVisual()], [UrdfCollision()])
l2 = UrdfLink("L2", [UrdfVisual()], [UrdfCollision()])

j = UrdfJoint("L1", "L2", "Joint")

editor.addLink(l1)
editor.addLink(l2)
editor.urdfJoints.append(j)

editor.createMultiBody(physicsClientId=gui)










p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
  p.stepSimulation(physicsClientId=gui)
  time.sleep(0.01)
