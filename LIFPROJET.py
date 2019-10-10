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

#id = p.loadURDF("test.urdf", useFixedBase=True)

b1 = Box("B1")
s1 = Sphere("S1")
b2 = Box("B2", [0,0,0.5])

j1 = UrdfJoint("B1", "S1", "B1S1", [0,0,0.5], [0,0,0], [1,0,0], p.JOINT_REVOLUTE)
j1.joint_lower_limit = -math.pi/4
j1.joint_upper_limit = math.pi/4

j2 = UrdfJoint("S1", "B2", "S1B2", [0,0,0], [0,0,0], [0,1,0], p.JOINT_REVOLUTE)
j2.joint_lower_limit = -math.pi/6
j2.joint_upper_limit = math.pi/6

editor.addLink(b1)
editor.addLink(s1)
editor.addLink(b2)

editor.addJoint(j1)
editor.addJoint(j2)

editor.writeLoad("test.urdf", [0,0,1], [0,0,0], True) 
editor.motorizeJoint("B1S1", targetVelocity = 1)

p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	if editor.getJointPosition("B1S1") <= -math.pi/5:
	    editor.motorizeJoint("B1S1", targetVelocity = 1)
	elif editor.getJointPosition("B1S1") >= math.pi/5:
		editor.motorizeJoint("B1S1", targetVelocity = -1)


	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)
