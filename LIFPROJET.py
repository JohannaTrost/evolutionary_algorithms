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

l1 = UrdfLink()
l1.link_name="L1"
l1.urdf_collision_shapes.append(UrdfCollision())
l1.urdf_visual_shapes.append(UrdfVisual())

editor.addLink(l1)
editor.urdfJoints.append(j)

editor.createMultiBody(physicsClientId=gui)








p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
  p.stepSimulation(physicsClientId=gui)
  time.sleep(0.01)
