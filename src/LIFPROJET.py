import pybullet as p
import pybullet_data
import time
import math
from Shapes import *
from Creature import *

from typing import Iterable, Tuple, Callable, T


def isInIntervalArray(data: Iterable[T], intervals: Iterable[Tuple[T, T]]) -> bool:
	for x, interval in zip(data, intervals):
		if not(interval[0] <= x <= interval[1]):
			return False
	return True

AnglePair = Tuple[float, float]
AngleList = Iterable[AnglePair]

"""
	AnglePair: Paire d'angles en radians (Y, Z),
	str: Parent limb name.
	str: Child limb name.
"""
MotorCallback = Callable[[AnglePair, str, str], None]

class MotorController(object):
	def __init__(self, targets: AngleList, func: MotorCallback, parentLimbName: str, childLimbName: str):
		self.positionTargets = targets
		self.func = func
		self.parentName = parentLimbName
		self.childName = childLimbName

		self._index = 0
	
	def start(self):
		self.func(self.positionTargets[self._index], self.parentName, self.childName)

	def update(self, currentAngle: AnglePair):
		angleErrorMargin = 0.5
		
		trgs = self.positionTargets[self._index]
		if isInIntervalArray(currentAngle, [(trgs[0] - angleErrorMargin, trgs[0] + angleErrorMargin), (trgs[1] - angleErrorMargin, trgs[1] + angleErrorMargin)]):
			if self._index + 1 >= len(self.positionTargets):
				self._index = 0
			else:
				self._index += 1

			self.func(self.positionTargets[self._index], self.parentName, self.childName)

##########################################
gui = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE),
p.createVisualShape(p.GEOM_PLANE))



def creatureControl(angles, parent, child):
	force = 200
	jointNames = c.getJointNames(parent, child)
	jointIndices = c.editor.getJointIndices(jointNames)

	for i, jointIndex in enumerate(jointIndices):
		p.setJointMotorControl2(c.editor.multiId, jointIndex, p.POSITION_CONTROL, targetPosition=angles[i], force=force, maxVelocity=1)



m = MotorController([
		(0, math.pi/3),
		(-math.pi / 3, 0), 
		(0, -math.pi/3),
		(math.pi / 3, 0)],
		creatureControl,
		"B1", "B2")


c = Creature("Creature", [0, 0, 3])

c.addLimb("", "B1", childOrigin=UrdfOrigin([0,0,0]))
c.addLimb("B1", "B2", UrdfOrigin([1,0,0]), UrdfOrigin([1,0,0]))
c.addLimb("B1", "B3", UrdfOrigin([-1,0,0]), UrdfOrigin([-1,0,0]))

c.load([0,0,0], True)

p.setRealTimeSimulation(1, physicsClientId=gui)

m.start()
while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	m.update(c.getAngleBetween("B1", "B2"))

	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)
