from urdfEditor import *
from Shapes import *
from Tree import *

import os
import numpy as np
import math

from typing import Tuple

class Limb(object):
	def __init__(self, extent=[1,1,1]):
		self.extent = extent

class Creature(object):
	def __init__(self, name="crr", spawnPosition=[0,0,0]):
		self.name = name
		self.editor = UrdfEditor(name)

		self.spawnPos = spawnPosition

	def load(self, orientation=[0,0,0], useFixedBase=False):
		scriptDir = os.path.dirname(__file__)
		relFilePath = f"..\\data\\{self.name}.urdf"

		self.editor.write_load(os.path.join(scriptDir, relFilePath), self.spawnPos, orientation, useFixedBase)

	def addLimb(self, parentLimbName: str, childLimbName: str, jointOrigin = UrdfOrigin(), childOrigin = UrdfOrigin(), extent = [1,1,1]):
		if len(self.editor.links) == 0:
			self.editor.add_link(Box(childLimbName, childOrigin, extent))
		else:
			jointNames = Creature.getJointNames(parentLimbName, childLimbName)
			sphere = Sphere(f"S_{parentLimbName}/{childLimbName}")
			previousLink = self.editor.get_link(parentLimbName)
			newLink = Box(childLimbName, childOrigin, extent)

			self.editor.add_link(sphere)
			self.editor.add_link(newLink)

			self.editor.add_joint(UrdfJointRevolute(
				parentLimbName, 
				sphere.name, 
				jointNames[0], 
				jointOrigin, [0,0,1]))
			self.editor.add_joint(UrdfJointRevolute(
				sphere.name,
				childLimbName, 
				jointNames[1], 
				UrdfOrigin([0,0,0], [0,0,0]), [0,1,0]))


	def getAngleBetween(self, parentLimbName: str , childLimbName: str):
		jointsNames = Creature.getJointNames(parentLimbName, childLimbName)
		return [
			self.editor.get_joint_position(jointsNames[0]),
			self.editor.get_joint_position(jointsNames[1])]

	def distFromStart(self):
		return np.linalg.norm((np.subtract(self.editor.getPosition(), self.spawnPos)))
	def mergeWith(self):
	    pass #TODO

	@staticmethod
	def getJointNames(parentLimbName: str, childLimbName: str) -> Tuple[str, str]:
		return (f"Jy_{parentLimbName}/{childLimbName}", f"Jz_{parentLimbName}/{childLimbName}")


def genRandomTree(root: Node, minNb:int, maxNb:int):
	def rec(node:Node, nbLimbsMax:int, nbLimbsLeft:int):
		nbAdded=0
		if nbLimbsLeft > 0:
			while nbAdded==0:
				nbAdded=np.random.randint(1, nbLimbsLeft+1)%nbLimbsMax

		nbLimbsLeftAfter=nbLimbsLeft-nbAdded

		for i in range(nbAdded):
			node.children.append(Node("Test"))
			nbLimbsLeftAfter=rec(node.children[i], nbLimbsMax, nbLimbsLeftAfter)

		return nbLimbsLeftAfter


	maxLimbsNb = np.random.randint(minNb, maxNb+1)
	rec(root, maxLimbsNb, maxLimbsNb)