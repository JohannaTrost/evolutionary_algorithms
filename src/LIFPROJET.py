import pybullet as p
import pybullet_data
import time
import math
from Shapes import *
from Creature import *

import lxml.etree as etree

def updateMovement(c: Creature, parentName: str, childName: str):
	angles = c.getAngleBetween(parentName, childName)
	max = math.pi/2
	min = -max
	speed = 3
	force = 500

	if angles[0] < min:
		c.motorizeVelocityY(parentName, childName, speed, force)
	elif angles[0] > max:
		c.motorizeVelocityY(parentName, childName, -speed, force)

	if angles[1] < min:
		c.motorizeVelocityZ(parentName, childName, speed, force)
	elif angles[1] > max:
		c.motorizeVelocityZ(parentName, childName, -speed, force)

##########################################
gui = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE), p.createVisualShape(p.GEOM_PLANE))


c = Creature("Creature", [0, 0, 3])

c.addLimb("", ChildLimbName="B1", childOrigin=UrdfOrigin([0,0,0]))
c.addLimb("B1", "B2", UrdfOrigin([1,0,0]), UrdfOrigin([1,0,0]))
c.addLimb("B1", "B3", UrdfOrigin([-1,0,0]), UrdfOrigin([-1,0,0]))

c.load([0,0,0])

c.motorizeVelocity("B1", "B2", 5, 5)
c.motorizeVelocity("B1", "B3", -5, -5)

p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	updateMovement(c, "B1", "B2")
	updateMovement(c, "B1", "B3")

	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)


tree=Node("root")

alea(tree, 3, 6)
tree.print()