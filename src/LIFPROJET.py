import pybullet as p
import pybullet_data
import time
import math
from Shapes import *
from Creature import *

import lxml.etree as etree

from typing import Iterable, Tuple, Callable


AnglePair = Tuple[float, float]
AngleList = Iterable[AnglePair]

"""
	AnglePair: Paire d'angles en radians (Y, Z),
	str: Parent limb name.
	str: Child limb name.
"""
MotorCallback = Callable[[AnglePair, str, str], None]

class MotorController(object):
	def __init__(self, targets: AngleList, func: MotorCallback, deltaTime = 1.0):
		self.positionTargets = targets
		self.func = func
		self.deltaTime = deltaTime
		
		self._timerStart = 0.0
		self._index = 0

	def Start(self):
		self._timerStart = time.time()
		
	
	def update(self, creature: Creature, parentLimbName: str, childLimbName: str):
		elapsed = time.time() - self._timerStart
		
		if elapsed >= self.deltaTime:
			if self._index + 1 >= len(self.positionTargets):
				self._index = 0
			else:
				self._index += 1			
			self.func(self.positionTargets[self._index], parentLimbName, childLimbName)
			
			self._timerStart = time.time()

##########################################
gui = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE),
p.createVisualShape(p.GEOM_PLANE))

def creatureControl(angles, parent, child):
	joints = c.getJointNames(parent, child)
	c.editor.motorizeJoint(joints[0], p.POSITION_CONTROL, targetPosition=angles[0], force=8)
	c.editor.motorizeJoint(joints[1], p.POSITION_CONTROL, targetPosition=angles[1], force=8)


m = MotorController([
		(0, math.pi/4), 
		(-math.pi / 4, 0), 
		(0, -math.pi/4), 
		(math.pi / 4, 0)
	],
		creatureControl,
		0.25
	)


c = Creature("Creature", [0, 0, 3])

c.addLimb("", childLimbName="B1", childOrigin=UrdfOrigin([0,0,0]))
c.addLimb("B1", "B2", UrdfOrigin([1,0,0]), UrdfOrigin([1,0,0]))
c.addLimb("B1", "B3", UrdfOrigin([-1,0,0]), UrdfOrigin([-1,0,0]))

c.load([0,0,0], True)

p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	m.update(c, "B1", "B2")

	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)



tree = Node("root")

genRandomTree(tree, 3, 6)
tree.print()