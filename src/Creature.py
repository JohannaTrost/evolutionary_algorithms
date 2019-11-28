from src.urdfEditor import *
from src.Shapes import *

import os
import numpy as np
import math


class Creature(object):
	def __init__(self, name="crr", spawnPosition=[0,0,0]):
		self.name = name
		self.editor = UrdfEditor(name)

		self.spawnPos = spawnPosition

	def load(self, orientation=[0,0,0], useFixedBase=False):
		scriptDir = os.path.dirname(__file__)
		relFilePath = f"..\\data\\{self.name}.urdf"

		self.editor.writeLoad(os.path.join(scriptDir, relFilePath), self.spawnPos, orientation, useFixedBase)

	def addLimb(self, parentLimbName: str, ChildLimbName: str, jointOrigin = UrdfOrigin(), childOrigin = UrdfOrigin(), extent = [1,1,1]):
		if len(self.editor.links) == 0:
		    self.editor.addLink(Box(ChildLimbName, childOrigin, extent))
		else:
			sphere = Sphere(f"S_{parentLimbName}/{ChildLimbName}")
			previousLink = self.editor.getLink(parentLimbName)
			newLink = Box(ChildLimbName, childOrigin, extent)

			self.editor.addLink(sphere)
			self.editor.addLink(newLink)

			self.editor.addJoint(UrdfJointRevolute(
				parentLimbName, 
				sphere.name, 
				f"Jz_{parentLimbName}/{ChildLimbName}", 
				jointOrigin, [0,0,1]))
			self.editor.addJoint(UrdfJointRevolute(
				sphere.name,
				ChildLimbName, 
				f"Jy_{parentLimbName}/{ChildLimbName}", 
				UrdfOrigin([0,0,0], [0,0,0]), [0,1,0]))

	def motorizeVelocityY(self, parentLimbName: str, childLimbName: str, velocity: int, force: int=100):
		self.editor.motorizeJoint(f"Jy_{parentLimbName}/{childLimbName}", targetVelocity=velocity, force=force)
	def motorizeVelocityZ(self, parentLimbName: str, childLimbName: str, velocity: int, force: int=100):
		self.editor.motorizeJoint(f"Jz_{parentLimbName}/{childLimbName}", targetVelocity=velocity, force=force)
	def motorizeVelocity(self, parentLimbName: str, childLimbName: str, velocityY: int=0, velocityZ: int=0, force: int=100):
		self.motorizeVelocityY(parentLimbName, childLimbName, velocityY, force)
		self.motorizeVelocityZ(parentLimbName, childLimbName, velocityZ, force)

	def getAngleBetween(self, parentLimbName: str , childLimbName: str):
		return [
			self.editor.getJointPosition(f"Jy_{parentLimbName}/{childLimbName}"), 
			self.editor.getJointPosition(f"Jz_{parentLimbName}/{childLimbName}")]

	def distFromStart(self):
		return np.linalg.norm((np.subtract(self.editor.getPosition(), self.spawnPos)))
	def mergeWith(self):
	    pass #TODO





class Node(object):
	def __init__(self, value):
		self.value=value
		self.children: List[Node] = []

	def print(self):
		self._printRec(0)

	def _printRec(self, padding:int):
		paddingStr = ""
		for _ in range(padding):
			paddingStr+='\t'

		print(f"{paddingStr}{self.value}")
		for child in self.children:
			child._printRec(padding+1)



def aleaRec(node:Node, nbLimbsMax:int, nbLimbsLeft:int):
	nbAdded=0
	if nbLimbsLeft > 0:
		while nbAdded==0:
		    nbAdded=np.random.randint(1, nbLimbsLeft+1)%nbLimbsMax

	nbLimbsLeftAfter=nbLimbsLeft-nbAdded

	for i in range(nbAdded):
		node.children.append(Node("Test"))
		nbLimbsLeftAfter=aleaRec(node.children[i], nbLimbsMax, nbLimbsLeftAfter)

	return nbLimbsLeftAfter

def alea(root: Node, minNb:int, maxNb:int):
	maxLimbsNb = np.random.randint(minNb, maxNb+1)
	aleaRec(root, maxLimbsNb, maxLimbsNb)