import pybullet as p
import pybullet_data
import time
import math
from src.Shapes import *
from src.Creature import *
import lxml.etree as etree


def updateMovement(c: Creature, parentName: str, childName: str):
	angles = c.getAngleBetween(parentName, childName)
	max = math.pi/2
	min = -max
	speed = 3
	force = 50

	if angles[0] < min:
		c.motorizeVelocityY(parentName, childName, speed, force)
	elif angles[0] > max:
		c.motorizeVelocityY(parentName, childName, -speed, force)

	if angles[1] < min:
		c.motorizeVelocityZ(parentName, childName, speed, force)
	elif angles[1] > max:
		c.motorizeVelocityZ(parentName, childName, -speed, force)


def updateMovementPos(c, parentName, childName, direction):
	step = math.pi / 50
	angles = c.getAngleBetween(parentName, childName)
	max = math.pi
	min = -max
	#step = math.pi/ 20
	#if angles[0] < min or angles[0] > max:
	#	direction[0] *= -1
	angles[0] += (step * direction[0])

	#if angles[1] < min or angles[1] > max:
	#	direction[1] *= -1
	angles[1] += (step * direction[1])

	p.resetJointStateMultiDof(c.editor.multiId, c.editor.jointNameToIndex[f"Jz_{parentName}/{childName}"], [angles[0]])
	p.resetJointStateMultiDof(c.editor.multiId, c.editor.jointNameToIndex[f"Jy_{parentName}/{childName}"], [angles[1]])

	return direction

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

c.motorizeVelocity("B1", "B2", 0, 2, 70)
c.motorizeVelocity("B1", "B3", 0, 2, 70)

p.setRealTimeSimulation(1, physicsClientId=gui)
direction = 1
counter = 0
direction = [1, 1]
direction2 = [1, 1]


while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	counter+=1
	if counter % 2 == 0:
		direction = updateMovementPos(c, "B1", "B2", direction)
		direction2 = updateMovementPos(c, "B1", "B3", direction2)
	#updateMovement(c, "B1", "B2")
	#updateMovement(c, "B1", "B3")

	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)


tree=Node("root")

alea(tree, 3, 6)
tree.print()