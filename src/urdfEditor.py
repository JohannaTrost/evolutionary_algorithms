import pybullet as p
import time
from math import pi

from lxml import etree
from lxml.builder import E
from typing import List


URDF_FLOAT_PREC = 5
def genFloat(value:float):
    return f'{value:.{URDF_FLOAT_PREC}f}'
def genVec(vec : List[float]):
    return " ".join(map(lambda f:genFloat(f), vec))  # map(), just for kicks

class UrdfSerializable(object):
	def toUrdf(self) -> etree.ElementBase:
		raise NotImplementedError()


class UrdfOrigin(UrdfSerializable):
	def __init__(self, origin_xyz=[0,0,0], origin_rpy=[0,0,0]):
		self.origin_xyz = origin_xyz
		self.origin_rpy = origin_rpy

	def toUrdf(self):
		return E.origin(rpy=genVec(self.origin_rpy), xyz=genVec(self.origin_xyz))


class UrdfInertial(UrdfSerializable):
	def __init__(self, origin=UrdfOrigin()):
		self.origin = origin
		self.mass = 1
		self.inertia_xxyyzz = [0, 0, 0]

	def toUrdf(self):
		return E.inertial(
			self.origin.toUrdf(),
			E.mass(value=genFloat(self.mass)),
			E.inertia(
				ixx=genFloat(self.inertia_xxyyzz[0]), 
				ixy="0", 
				ixz="0", 
				iyy=genFloat(self.inertia_xxyyzz[1]), 
				iyz="0", 
				izz=genFloat(self.inertia_xxyyzz[2])
				)
			)


class UrdfContact(object):

	def __init__(self, lateral_friction=1, rolling_friction=0, spinning_friction=0):
		self.lateral_friction = lateral_friction
		self.rolling_friction = rolling_friction
		self.spinning_friction = spinning_friction




class UrdfGeometry(UrdfSerializable):
	def toUrdf(self):
		return E.geometry()
class UrdfBox(UrdfGeometry):
	def __init__(self, extent=[1,1,1]):
		self.extent = extent

	def toUrdf(self):
		xmlBox = super().toUrdf()
		xmlBox.append(E.box(size=genVec(self.extent)))
		return xmlBox
class UrdfSphere(UrdfGeometry):
	def __init__(self, radius=0.5):
		self.radius = radius

	def toUrdf(self):
		xmlSphere = super().toUrdf()
		xmlSphere.append(E.sphere(radius=genFloat(self.radius)))
		return xmlSphere


class UrdfMaterial(UrdfSerializable):
	def __init__(self, name="default", colorRGBA=[1.0, 0.0, 0.0, 1.0]):
		self.name = name
		self.colorRGBA = colorRGBA

	def toUrdf(self):
		return E.material(
			E.color(rgba=genVec(self.colorRGBA)),

			name=self.name
		)

class UrdfVisual(UrdfSerializable):
	def __init__(self, origin=UrdfOrigin(), geometry=UrdfBox(), material=UrdfMaterial()):
		self.origin = origin
		self.geometry=geometry
		self.material=material

	def toUrdf(self):
		return E.visual(
			self.origin.toUrdf(),
			self.geometry.toUrdf(),
			self.material.toUrdf()
			)

class UrdfCollision(UrdfSerializable):
	def __init__(self, origin=UrdfOrigin(), geometry=UrdfBox()):
		self.origin=origin
		self.geometry=geometry

	def toUrdf(self):
		return E.collision(
			self.origin.toUrdf(),
			self.geometry.toUrdf()
			)


class UrdfLink(UrdfSerializable):
	def __init__(self, name = "dummy", collision=UrdfCollision(), visual=UrdfVisual(), inertial=UrdfInertial()):
		self.name=name
		self.collision=collision
		self.visual=visual
		self.inertial=inertial

	def toUrdf(self):
		return E.link(
			self.inertial.toUrdf(),
			self.visual.toUrdf(),
			self.collision.toUrdf(),

			name=self.name
			)



class UrdfJoint(UrdfSerializable):
	def __init__(self, 
			  parent_name, 
			  child_name, 
			  name = "joint_dummy",
			  origin = UrdfOrigin()
			):

		self.parent_name = parent_name
		self.child_name = child_name
		self.name = name
		self.origin = origin

	def toUrdf(self):
		return E.joint(
			E.parent(link=self.parent_name),
			E.child(link=self.child_name),
			E.dynamics(damping="1.0", friction="0.0001"),
			self.origin.toUrdf(),

			name = self.name
			)

class UrdfJointRevolute(UrdfJoint):
	def __init__(self, 
			  parent_name, 
			  child_name, 
			  name = "joint_dummy",
			  origin = UrdfOrigin(),
			  axis_xyz=[0,0,1],
			  lower_limit=-pi,
			  upper_limit=pi):
		super().__init__(parent_name, child_name, name, origin)

		self.axis_xyz = axis_xyz
		self.lower_limit = lower_limit
		self.upper_limit = upper_limit

	def toUrdf(self):
		xmlJoint = super().toUrdf()

		xmlJoint.set("type", "revolute")
		xmlJoint.append(E.axis(xyz=genVec(self.axis_xyz)))
		xmlJoint.append(E.limit(effort="1000.0", lower=genFloat(self.lower_limit), upper=genFloat(self.upper_limit), velocity="0.5"))
		
		return xmlJoint
class UrdfJointContinuous(UrdfJoint):
	def __init__(self, 
			  parent_name,
			  child_name, 
			  name = "joint_dummy", 
			  origin = UrdfOrigin(),
			  axis_xyz=[0,0,1]):
		super().__init__(parent_name, child_name, name, origin)

		self.axis_xyz = axis_xyz

	def toUrdf(self):
		xmlJoint = super().toUrdf()

		xmlJoint.set("type", "continuous")
		xmlJoint.append(E.axis(xyz=genVec(self.axis_xyz)))
		return xmlJoint






class UrdfEditor(object):

	def __init__(self, robotName="robot"):
		self.robotName = robotName

		self.multiId = -1
		self.links = []
		self.joints = []		
		self.linkNameToIndex = {}
		self.jointNameToIndex = {}

	def saveUrdf(self, fileName, saveVisuals=True):
		root = E.robot(name=self.robotName)

		for link in self.links:
			root.append(link.toUrdf())
		for joint in self.joints:
			root.append(joint.toUrdf())

		with open(fileName, "wb+") as f:
			tstr=etree.tostring(root, xml_declaration=True, encoding="utf-8", pretty_print=True)
			f.write(tstr)
	
	def addLink(self, link: UrdfLink):
		self.linkNameToIndex[link.name] = len(self.links)
		self.links.append(link)
	def addJoint(self, joint: UrdfJoint):
		self.jointNameToIndex[joint.name] = len(self.joints)
		self.joints.append(joint)

	def getLink(self, name: str):
		return self.links[self.linkNameToIndex[name]]
	def getJoint(self, name: str):
		return self.joints[self.jointNameToIndex[name]]
	
	def writeLoad(self, pathToSave, position=[0,0,0], orientation=[0,0,0], useFixedBase=False):
		self.saveUrdf(pathToSave)
		self.multiId = p.loadURDF(pathToSave, position, p.getQuaternionFromEuler(orientation), useFixedBase=useFixedBase)

	def getBasePosition(self):
		return p.getBasePositionAndOrientation(self.multiId)[0]


	def motorizeJoint(self, jointName: str, controlMode=p.VELOCITY_CONTROL, targetPosition=0, targetVelocity=0, force=2):
		p.setJointMotorControl2(self.multiId, self.jointNameToIndex[jointName], controlMode,
								targetPosition=targetPosition, targetVelocity=targetVelocity, force=force)
	def getJointPosition(self, name: str):
		return p.getJointState(self.multiId, self.jointNameToIndex[name])[0]