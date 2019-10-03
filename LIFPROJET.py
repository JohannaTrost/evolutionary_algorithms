import pybullet as p
import pybullet_data
import time
from urdfEditor import *

##########################################
gui = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE), p.createVisualShape(p.GEOM_PLANE))




editor = UrdfEditor()

l1 = UrdfLink("L1", 
			  [UrdfVisual([0,0,1])], 
			  [UrdfCollision([0,0,1])]
			  )
l2 = UrdfLink("L2", 
			  [UrdfVisual([1,0,1])], 
			  [UrdfCollision([1,0,1])]
			  )

j = UrdfJoint("L1", "L2", l2, "Joint")

editor.addLink(l1)
editor.addLink(l2)
editor.urdfJoints.append(j)

editor.createMultiBody(physicsClientId=gui)


editor.saveUrdf("test.urdf")







p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
  p.stepSimulation(physicsClientId=gui)
  time.sleep(0.01)
