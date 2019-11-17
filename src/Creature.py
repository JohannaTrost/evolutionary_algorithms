from urdfEditor import *
from Shapes import *

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
				f"Jx_{parentLimbName}/{ChildLimbName}", 
				jointOrigin, [1,0,0]))
			self.editor.addJoint(UrdfJointRevolute(
				sphere.name, 
				ChildLimbName, 
				f"Jy_{parentLimbName}/{ChildLimbName}", 
				UrdfOrigin([0,0,0], [0,0,0]), [0,1,0]))

	def distFromStart(self):
		return np.linalg.norm((np.subtract(self.editor.getPosition(), self.spawnPos)))
	def mergeWith(self):

	    pass #TODO