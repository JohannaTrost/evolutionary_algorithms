from urdfEditor import *
from Shapes import *

import os

class Creature(object):
	def __init__(self, name="crr"):
		self.name = name
		self.editor = UrdfEditor(name)
		self.editor.addLink(Box("Base"))

	def load(self, position = [0,0,0], orientation=[0,0,0], useFixedBase=False):
		scriptDir = os.path.dirname(__file__)
		relFilePath = f"..\\data\\{self.name}.urdf"

		self.editor.writeLoad(os.path.join(scriptDir, relFilePath), position, orientation, useFixedBase)

	def addLimb(self, parentLimbName: str, newChildLimbName: str, jointOrigin = [0,0,0], jointOrientation = [0,0,0], childOrigin = [0,0,0], extent = [1,1,1]):
		sphere = Sphere(f"S_{parentLimbName}/{newChildLimbName}")
		previousLink = self.editor.getLink(parentLimbName)
		newLink = Box(newChildLimbName, UrdfOrigin(childOrigin, [0,0,0]), extent)

		self.editor.addLink(sphere)
		self.editor.addLink(newLink)

		self.editor.addJoint(UrdfJointRevolute(
			parentLimbName, 
			sphere.name, 
			f"Jx_{parentLimbName}/{newChildLimbName}", 
			UrdfOrigin(jointOrigin, jointOrientation), [1,0,0]))
		self.editor.addJoint(UrdfJointRevolute(
			sphere.name, 
			newChildLimbName, 
			f"Jy_{parentLimbName}/{newChildLimbName}", 
			UrdfOrigin([0,0,0], [0,0,0]), [0,1,0]))

	def mergeWith():
	    pass #TODO