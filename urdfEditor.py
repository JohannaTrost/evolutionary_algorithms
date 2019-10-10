import pybullet as p
import time

from typing import List

def genVec(vec : List[float], precision = 5):
    return " ".join(map(lambda f:f'{f:.{precision}f}', vec))  # map(), just for kicks



class UrdfInertial(object):

	def __init__(self):
		self.mass = 1
		self.inertia_xxyyzz = [0, 0, 0]
		self.origin_rpy = [0, 0, 0]
		self.origin_xyz = [0, 0, 0]


class UrdfContact(object):

	def __init__(self, lateral_friction = 1, rolling_friction = 0, spinning_friction = 0):
		self.lateral_friction = lateral_friction
		self.rolling_friction = rolling_friction
		self.spinning_friction = spinning_friction


class UrdfLink(object):

	def __init__(self, name = "dummy", visual_shapes = [], collision_shapes = []):
		self.link_name = name
		self.urdf_inertial = UrdfInertial()
		self.urdf_visual_shapes = visual_shapes
		self.urdf_collision_shapes = collision_shapes


class UrdfVisual(object):

	def __init__(self, origin_xyz = [0,0,0], origin_rpy = [0,0,0], meshfilename = "meshfile"):
		self.origin_rpy = origin_rpy
		self.origin_xyz = origin_xyz
		self.geom_type = p.GEOM_BOX
		self.geom_radius = 1
		self.geom_extents = [1,1,1]
		self.geom_length = 10
		self.geom_meshfilename = meshfilename
		self.geom_meshscale = [1, 1, 1]
		self.material_rgba = [1, 0, 0, 1]
		self.material_name = ""


class UrdfCollision(object):

	def __init__(self, origin_xyz = [0,0,0], origin_rpy = [0,0,0], meshfilename = "meshfile"):
		self.origin_rpy = origin_rpy
		self.origin_xyz = origin_xyz
		self.geom_type = p.GEOM_BOX
		self.geom_radius = 1
		self.geom_length = 2
		self.geom_extents = [1, 1, 1]
		self.geom_meshfilename = meshfilename
		self.geom_meshscale = [1, 1, 1]


class UrdfJoint(object):

	def __init__(self, parent_name, child_name, joint_name = "joint_dummy", joint_origin_xyz = [0,0,0], joint_origin_rpy = [0,0,0], joint_axis_xyz = [0,0,1], joint_type = p.JOINT_REVOLUTE):
		self.joint_name = joint_name
		self.joint_type = joint_type
		self.joint_lower_limit = 0
		self.joint_upper_limit = -1
		self.parent_name = parent_name
		self.child_name = child_name
		self.joint_origin_xyz = joint_origin_xyz
		self.joint_origin_rpy = joint_origin_rpy
		self.joint_axis_xyz = joint_axis_xyz


class UrdfEditor(object):

	def __init__(self):
		self.initialize()
	def __del__(self):
		pass




	def initialize(self):
		self.multiId = -1;
		self.urdfLinks = []
		self.urdfJoints = []
		self.robotName = ""
		self.linkNameToIndex = {}
		self.jointNameToIndex = {}

	def convertLinkFromMultiBody(self, bodyUid, linkIndex, urdfLink, physicsClientId):
		dyn = p.getDynamicsInfo(bodyUid, linkIndex, physicsClientId=physicsClientId)
		urdfLink.urdf_inertial.mass = dyn[0]
		urdfLink.urdf_inertial.inertia_xxyyzz = dyn[2]
		#todo
		urdfLink.urdf_inertial.origin_xyz = dyn[3]
		rpy = p.getEulerFromQuaternion(dyn[4])
		urdfLink.urdf_inertial.origin_rpy = rpy

		visualShapes = p.getVisualShapeData(bodyUid, physicsClientId=physicsClientId)
		matIndex = 0
		for v in visualShapes:
			if (v[1] == linkIndex):
				urdfVisual = UrdfVisual()
				urdfVisual.geom_type = v[2]
				if (v[2] == p.GEOM_BOX):
					urdfVisual.geom_extents = v[3]
				if (v[2] == p.GEOM_SPHERE):
					urdfVisual.geom_radius = v[3][0]
				if (v[2] == p.GEOM_MESH):
					urdfVisual.geom_meshfilename = v[4].decode("utf-8")
					urdfVisual.geom_meshscale = v[3]
				if (v[2] == p.GEOM_CYLINDER):
					urdfVisual.geom_length = v[3][0]
					urdfVisual.geom_radius = v[3][1]
				if (v[2] == p.GEOM_CAPSULE):
					urdfVisual.geom_length = v[3][0]
					urdfVisual.geom_radius = v[3][1]
				urdfVisual.origin_xyz = v[5]
				urdfVisual.origin_rpy = p.getEulerFromQuaternion(v[6])
				urdfVisual.material_rgba = v[7]
				name = 'mat_{}_{}'.format(linkIndex, matIndex)
				urdfVisual.material_name = name
				urdfLink.urdf_visual_shapes.append(urdfVisual)
				matIndex = matIndex + 1

		collisionShapes = p.getCollisionShapeData(bodyUid, linkIndex, physicsClientId=physicsClientId)
		for v in collisionShapes:
			urdfCollision = UrdfCollision()
			urdfCollision.geom_type = v[2]
			if (v[2] == p.GEOM_BOX):
				urdfCollision.geom_extents = v[3]
			if (v[2] == p.GEOM_SPHERE):
				urdfCollision.geom_radius = v[3][0]
			if (v[2] == p.GEOM_MESH):
				urdfCollision.geom_meshfilename = v[4].decode("utf-8")
				urdfCollision.geom_meshscale = v[3]
			if (v[2] == p.GEOM_CYLINDER):
				urdfCollision.geom_length = v[3][0]
				urdfCollision.geom_radius = v[3][1]
			if (v[2] == p.GEOM_CAPSULE):
				urdfCollision.geom_length = v[3][0]
				urdfCollision.geom_radius = v[3][1]
			pos,orn = p.multiplyTransforms(dyn[3],dyn[4],\
			 v[5], v[6])
			urdfCollision.origin_xyz = pos
			urdfCollision.origin_rpy = p.getEulerFromQuaternion(orn)
			urdfLink.urdf_collision_shapes.append(urdfCollision)

	def initializeFromBulletBody(self, bodyUid, physicsClientId):
		self.initialize()

		#always create a base link
		baseLink = UrdfLink()
		baseLinkIndex = -1
		self.convertLinkFromMultiBody(bodyUid, baseLinkIndex, baseLink, physicsClientId)
		baseLink.link_name = p.getBodyInfo(bodyUid, physicsClientId=physicsClientId)[0].decode("utf-8")
		self.linkNameToIndex[baseLink.link_name] = len(self.urdfLinks)
		self.urdfLinks.append(baseLink)

		#optionally create child links and joints
		for j in range(p.getNumJoints(bodyUid, physicsClientId=physicsClientId)):
			jointInfo = p.getJointInfo(bodyUid, j, physicsClientId=physicsClientId)
			urdfLink = UrdfLink()
			self.convertLinkFromMultiBody(bodyUid, j, urdfLink, physicsClientId)
			urdfLink.link_name = jointInfo[12].decode("utf-8")
			self.linkNameToIndex[urdfLink.link_name] = len(self.urdfLinks)
			self.urdfLinks.append(urdfLink)

			urdfJoint = UrdfJoint()
			urdfJoint.link = urdfLink
			urdfJoint.joint_name = jointInfo[1].decode("utf-8")
			urdfJoint.joint_type = jointInfo[2]
			urdfJoint.joint_axis_xyz = jointInfo[13]
			orgParentIndex = jointInfo[16]
			if (orgParentIndex < 0):
				urdfJoint.parent_name = baseLink.link_name
			else:
				parentJointInfo = p.getJointInfo(bodyUid, orgParentIndex, physicsClientId=physicsClientId)
				urdfJoint.parent_name = parentJointInfo[12].decode("utf-8")
			urdfJoint.child_name = urdfLink.link_name

			#todo, compensate for inertia/link frame offset

			dynChild = p.getDynamicsInfo(bodyUid, j, physicsClientId=physicsClientId)
			childInertiaPos = dynChild[3]
			childInertiaOrn = dynChild[4]
			parentCom2JointPos = jointInfo[14]
			parentCom2JointOrn = jointInfo[15]
			tmpPos, tmpOrn = p.multiplyTransforms(childInertiaPos, childInertiaOrn, parentCom2JointPos,
																						parentCom2JointOrn)
			tmpPosInv, tmpOrnInv = p.invertTransform(tmpPos, tmpOrn)
			dynParent = p.getDynamicsInfo(bodyUid, orgParentIndex, physicsClientId=physicsClientId)
			parentInertiaPos = dynParent[3]
			parentInertiaOrn = dynParent[4]

			pos, orn = p.multiplyTransforms(parentInertiaPos, parentInertiaOrn, tmpPosInv, tmpOrnInv)
			pos, orn_unused = p.multiplyTransforms(parentInertiaPos, parentInertiaOrn,
																						 parentCom2JointPos, [0, 0, 0, 1])

			urdfJoint.joint_origin_xyz = pos
			urdfJoint.joint_origin_rpy = p.getEulerFromQuaternion(orn)

			self.urdfJoints.append(urdfJoint)

	def writeInertial(self, urdfInertial, precision=5):
		str = f'		<inertial>\n'
		str += f'			<origin rpy="{genVec(urdfInertial.origin_rpy, precision)}" xyz="{genVec(urdfInertial.origin_xyz, precision)}"/>\n'
		str += f'			<mass value="{round(urdfInertial.mass, precision)}"/>\n'
		str += f'			<inertia ixx="{round(urdfInertial.inertia_xxyyzz[0], precision)}" ixy="0" ixz="0" iyy="{round(urdfInertial.inertia_xxyyzz[1], precision)}" iyz="0" izz="{round(urdfInertial.inertia_xxyyzz[2], precision)}"/>\n'
		str += f'		</inertial>\n'

		return str

	def writeVisualShape(self, urdfVisual, precision=5):
		#we don't support loading capsule types from visuals, so auto-convert from
		#collision shape
		if urdfVisual.geom_type == p.GEOM_CAPSULE:
			return

		str = f'		<visual>\n'
		str += f'			<origin rpy="{genVec(urdfVisual.origin_rpy, precision)}" xyz="{genVec(urdfVisual.origin_xyz, precision)}"/>\n'

		str += f'			<geometry>\n'
		if urdfVisual.geom_type == p.GEOM_BOX:
			str += f'				<box size="{genVec(urdfVisual.geom_extents, precision)}"/>\n'

		if urdfVisual.geom_type == p.GEOM_SPHERE:
			str += f'				<sphere radius="{round(urdfVisual.geom_radius, precision)}"/>\n'

		if urdfVisual.geom_type == p.GEOM_MESH:
			str += f'				<mesh filename="{urdfVisual.geom_meshfilename}" scale="{genVec(urdfVisual.geom_meshscale, precision)}"/>\n'

		if urdfVisual.geom_type == p.GEOM_CYLINDER:
			str += f'				<cylinder length="{round(urdfVisual.geom_length, precision)}" radius="{round(urdfVisual.geom_radius, precision)}"/>\n'

		if urdfVisual.geom_type == p.GEOM_CAPSULE:
			str += f'				<capsule length="{round(urdfVisual.geom_length, precision)}" radius="{round(urdfVisual.geom_radius, precision)}"/>\n'

		str += f'			</geometry>\n'
		str += f'			<material name="{urdfVisual.material_name}">\n'
		str += f'				<color rgba="{genVec(urdfVisual.material_rgba, precision)}" />\n'
		str += f'			</material>\n'
		str += f'		</visual>\n'

		return str

	def writeCollisionShape(self, urdfCollision, precision=5):
		str = f'		<collision>\n'
		str += f'			<origin rpy="{genVec(urdfCollision.origin_rpy, precision)}" xyz="{genVec(urdfCollision.origin_xyz, precision)}"/>\n'

		str += f'			<geometry>\n'
		if urdfCollision.geom_type == p.GEOM_BOX:
			str += f'				<box size="{genVec(urdfCollision.geom_extents, precision)}"/>\n'

		if urdfCollision.geom_type == p.GEOM_SPHERE:
			str += f'				<sphere radius="{round(urdfCollision.geom_radius, precision)}"/>\n'

		if urdfCollision.geom_type == p.GEOM_MESH:
			str += f'				<mesh filename="{urdfCollision.geom_meshfilename}" scale="{genVec(urdfCollision.geom_meshscale, precision)}"/>\n'

		if urdfCollision.geom_type == p.GEOM_CYLINDER:
			str += f'				<cylinder length="{round(urdfCollision.geom_length, precision)}" radius="{round(urdfCollision.geom_radius, precision)}"/>\n'

		if urdfCollision.geom_type == p.GEOM_CAPSULE:
			str += f'				<capsule length="{round(urdfCollision.geom_length, precision)}" radius="{round(urdfCollision.geom_radius, precision)}"/>\n'

		str += f'			</geometry>\n'
		str += f'		</collision>\n'

		return str

	def writeLink(self, urdfLink, saveVisuals):
		str = f'	<link name="{urdfLink.link_name}">\n'
		str += self.writeInertial(urdfLink.urdf_inertial)

		hasCapsules = False
		for v in urdfLink.urdf_visual_shapes:
			if (v.geom_type == p.GEOM_CAPSULE):
				hasCapsules = True
		if (saveVisuals and not hasCapsules):
			for v in urdfLink.urdf_visual_shapes:
				str += self.writeVisualShape(v)
		for c in urdfLink.urdf_collision_shapes:
			str += self.writeCollisionShape(c)
		str += f'	</link>\n'

		return str

	def writeJoint(self, urdfJoint, precision=5):
		str = ''
		jointTypeStr = "invalid"

		if urdfJoint.joint_type == p.JOINT_REVOLUTE:
			if urdfJoint.joint_upper_limit < urdfJoint.joint_lower_limit:
				jointTypeStr = "continuous"
			else:
				jointTypeStr = "revolute"
		elif urdfJoint.joint_type == p.JOINT_FIXED:
			jointTypeStr = "fixed"
		elif urdfJoint.joint_type == p.JOINT_PRISMATIC:
			jointTypeStr = "prismatic"
		elif urdfJoint.joint_type == p.JOINT_SPHERICAL:
			jointTypeStr = "spherical"
			
		str += f'<joint name="{urdfJoint.joint_name}" type="{jointTypeStr}">\n'
		str += f'		<parent link="{urdfJoint.parent_name}"/>\n'
		str += f'		<child link="{urdfJoint.child_name}"/>\n'

		if urdfJoint.joint_type == p.JOINT_PRISMATIC:
			#todo: handle limits
			lowerLimit = -0.5
			upperLimit = 0.5
			str += f'		<limit effort="1000.0" lower="{round(lowerLimit, precision)}" upper="{round(upperLimit, precision)}" velocity="0.5"/>\n'
		if jointTypeStr == "revolute":
			str += f'		<limit effort="1000.0" lower="{round(urdfJoint.joint_lower_limit, precision)}" upper="{round(urdfJoint.joint_upper_limit, precision)}" velocity="0.5"/>\n'


		str += f'		<dynamics damping="1.0" friction="0.0001"/>\n'
		str += f'		<origin rpy="{genVec(urdfJoint.joint_origin_rpy, precision)}" xyz="{genVec(urdfJoint.joint_origin_xyz, precision)}"/>\n'
		if urdfJoint.joint_type != p.JOINT_SPHERICAL:
			str += f'		<axis xyz="{genVec(urdfJoint.joint_axis_xyz, precision)}"/>\n'
		str += f'	</joint>\n'

		return str

	def saveUrdf(self, fileName, saveVisuals=True):
		file = open(fileName, "w")
		str = ""
		str += f'<?xml version="1.0" ?>\n'
		str += f'<robot name="{self.robotName}">'
		
		for link in self.urdfLinks:
			str += self.writeLink(link, saveVisuals)

		for joint in self.urdfJoints:
			str += self.writeJoint(joint)

		str += f'</robot>\n'

		file.write(str)
		file.close()

	def joinUrdf(self,
							 childEditor,
							 parentLinkIndex=0,
							 jointPivotXYZInParent=[0, 0, 0],
							 jointPivotRPYInParent=[0, 0, 0],
							 jointPivotXYZInChild=[0, 0, 0],
							 jointPivotRPYInChild=[0, 0, 0],
							 parentPhysicsClientId=0,
							 childPhysicsClientId=0):

		childLinkIndex = len(self.urdfLinks)
		insertJointIndex = len(self.urdfJoints)

		#combine all links, and add a joint

		for link in childEditor.urdfLinks:
			self.linkNameToIndex[link.link_name] = len(self.urdfLinks)
			self.urdfLinks.append(link)
		for joint in childEditor.urdfJoints:
			self.urdfJoints.append(joint)
		#add a new joint between a particular

		jointPivotQuatInChild = p.getQuaternionFromEuler(jointPivotRPYInChild)
		invJointPivotXYZInChild, invJointPivotQuatInChild = p.invertTransform(jointPivotXYZInChild, jointPivotQuatInChild)

		#apply this invJointPivot***InChild to all inertial, visual and collision
		#element in the child link
		#inertial
		pos, orn = p.multiplyTransforms(self.urdfLinks[childLinkIndex].urdf_inertial.origin_xyz,
																		p.getQuaternionFromEuler(self.urdfLinks[childLinkIndex].urdf_inertial.origin_rpy),
																		invJointPivotXYZInChild,
																		invJointPivotQuatInChild,
																		physicsClientId=parentPhysicsClientId)
		self.urdfLinks[childLinkIndex].urdf_inertial.origin_xyz = pos
		self.urdfLinks[childLinkIndex].urdf_inertial.origin_rpy = p.getEulerFromQuaternion(orn)
		#all visual
		for v in self.urdfLinks[childLinkIndex].urdf_visual_shapes:
			pos, orn = p.multiplyTransforms(v.origin_xyz, p.getQuaternionFromEuler(v.origin_rpy),
																			invJointPivotXYZInChild, invJointPivotQuatInChild)
			v.origin_xyz = pos
			v.origin_rpy = p.getEulerFromQuaternion(orn)
		#all collision
		for c in self.urdfLinks[childLinkIndex].urdf_collision_shapes:
			pos, orn = p.multiplyTransforms(c.origin_xyz, p.getQuaternionFromEuler(c.origin_rpy),
																			invJointPivotXYZInChild, invJointPivotQuatInChild)
			c.origin_xyz = pos
			c.origin_rpy = p.getEulerFromQuaternion(orn)

		childLink = self.urdfLinks[childLinkIndex]
		parentLink = self.urdfLinks[parentLinkIndex]

		joint = UrdfJoint()
		joint.link = childLink
		joint.joint_name = "joint_dummy1"
		joint.joint_type = p.JOINT_REVOLUTE
		joint.joint_lower_limit = 0
		joint.joint_upper_limit = -1
		joint.parent_name = parentLink.link_name
		joint.child_name = childLink.link_name
		joint.joint_origin_xyz = jointPivotXYZInParent
		joint.joint_origin_rpy = jointPivotRPYInParent
		joint.joint_axis_xyz = [0, 0, 1]

		#the following commented line would crash PyBullet, it messes up the joint
		#indexing/ordering
		#self.urdfJoints.append(joint)

		#so make sure to insert the joint in the right place, to links/joints match
		self.urdfJoints.insert(insertJointIndex, joint)
		return joint

	#def createMultiBody(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], physicsClientId=0):
	#	#assume link[0] is base
	#	if (len(self.urdfLinks) == 0):
	#		return -1

	#	#for i in range (len(self.urdfLinks)):
	#	#	print("link", i, "=",self.urdfLinks[i].link_name)
		
	#	createdLinks = {}

	#	base = self.urdfLinks[0]

	#	#v.tmp_collision_shape_ids=[]
	#	baseMass = base.urdf_inertial.mass
	#	baseShapes = createShapes(base)
	#	createdLinks[base.link_name] = baseShapes

	#	linkMasses = []
	#	linkCollisionShapeIndices = []
	#	linkVisualShapeIndices = []
	#	linkPositions = []
	#	linkOrientations = []

	#	linkInertialFramePositions = []
	#	linkInertialFrameOrientations = []
	#	linkParentIndices = []
	#	linkJointTypes = []
	#	linkJointAxis = []


	#	for joint in self.urdfJoints:
	#		linkParentIndex = self.linkNameToIndex[joint.parent_name]
	#		link = self.urdfLinks[self.linkNameToIndex[joint.child_name]]

	#		shapeIndices = []
	#		if link.link_name in createdLinks:
	#			shapeIndices = createdLinks[link.link_name]
	#		else:
	#			shapeIndices = createShapes(link)
	#			createdLinks[link.link_name] = shapeIndices

			
	#		linkCollisionShapeIndices.append(shapeIndices[0])
	#		linkVisualShapeIndices.append(shapeIndices[1])
	#		linkMasses.append(link.urdf_inertial.mass)
	#		linkPositions.append(joint.joint_origin_xyz)
	#		linkOrientations.append(p.getQuaternionFromEuler(joint.joint_origin_rpy))
	#		linkInertialFramePositions.append(link.urdf_inertial.origin_xyz)
	#		linkInertialFrameOrientations.append(p.getQuaternionFromEuler(link.urdf_inertial.origin_rpy))
	#		linkParentIndices.append(linkParentIndex)
	#		linkJointTypes.append(joint.joint_type)
	#		linkJointAxis.append(joint.joint_axis_xyz)
			
			
	#	obUid = p.createMultiBody(baseMass,\
	#		baseCollisionShapeIndex=baseShapes[0],
	#		baseVisualShapeIndex=baseShapes[1],
	#		basePosition=basePosition,
	#		baseOrientation=baseOrientation,
	#		baseInertialFramePosition=base.urdf_inertial.origin_xyz,
	#		baseInertialFrameOrientation=p.getQuaternionFromEuler(base.urdf_inertial.origin_rpy),
			
	#		linkMasses=linkMasses,
	#		linkCollisionShapeIndices=linkCollisionShapeIndices,
	#		linkVisualShapeIndices=linkVisualShapeIndices,
	#		linkPositions=linkPositions,
	#		linkOrientations=linkOrientations,
	#		linkInertialFramePositions=linkInertialFramePositions,
	#		linkInertialFrameOrientations=linkInertialFrameOrientations,
	#		linkParentIndices=linkParentIndices,
	#		linkJointTypes=linkJointTypes,
	#		linkJointAxis=linkJointAxis,
	#		#flags = p.URDF_USE_SELF_COLLISION,
	#		physicsClientId=physicsClientId)
	#	return obUid
	
	def addLink(self, link: UrdfLink):
		self.linkNameToIndex[link.link_name] = len(self.urdfLinks)
		self.urdfLinks.append(link)
	def addJoint(self, joint: UrdfJoint):
		self.jointNameToIndex[joint.joint_name] = len(self.urdfJoints)
		self.urdfJoints.append(joint)
	
	def writeLoad(self, pathToSave: str, position=[0,0,0], orientation=[0,0,0], useFixedBase = False):
		self.saveUrdf(pathToSave)
		self.id = p.loadURDF(pathToSave, position, p.getQuaternionFromEuler(orientation), useFixedBase = useFixedBase)

	def motorizeJoint(self, jointName: str, controlMode = p.VELOCITY_CONTROL, targetPosition = 0, targetVelocity = 0, force = 10):
		p.setJointMotorControl2(self.id, self.jointNameToIndex[jointName], controlMode, targetPosition = targetPosition, targetVelocity = targetVelocity, force = force)
		
	def getJointPosition(self, name: str):
		return p.getJointState(self.id, self.jointNameToIndex[name])[0]

#def createCollisionShape(collision_shapes: List[UrdfCollision], physicsClientId = 0):
		
#	CollisionShapeIndex = -1
#	ShapeTypeArray = [v.geom_type for v in collision_shapes]
#	RadiusArray = [v.geom_radius for v in collision_shapes]
#	HalfExtentsArray = [[ext * 0.5 for ext in v.geom_extents] for v in collision_shapes]
#	lengthsArray = [v.geom_length for v in collision_shapes]
#	fileNameArray = [v.geom_meshfilename for v in collision_shapes]
#	meshScaleArray = [v.geom_meshscale for v in collision_shapes]
#	PositionsArray = [v.origin_xyz for v in collision_shapes]
#	OrientationsArray = [p.getQuaternionFromEuler(v.origin_rpy) for v in collision_shapes]

#	if (len(ShapeTypeArray)):
#		#print("fileNameArray=",fileNameArray)
#		CollisionShapeIndex = p.createCollisionShapeArray(shapeTypes=ShapeTypeArray,
#				radii=RadiusArray,
#				halfExtents=HalfExtentsArray,
#				lengths=lengthsArray,
#				fileNames=fileNameArray,
#				meshScales=meshScaleArray,
#				collisionFramePositions=PositionsArray,
#				collisionFrameOrientations=OrientationsArray,
#				physicsClientId=physicsClientId)

#	return CollisionShapeIndex

#def createVisualShape(visual_shapes: List[UrdfVisual], physicsClientId=0):

#	VisualShapeIndex = -1
#	shapeTypes = [v.geom_type for v in visual_shapes]
#	halfExtents = [[ext * 0.5 for ext in v.geom_extents] for v in visual_shapes]
#	radii = [v.geom_radius for v in visual_shapes]
#	lengths = [v.geom_length for v in visual_shapes]
#	fileNames = [v.geom_meshfilename for v in visual_shapes]
#	meshScales = [v.geom_meshscale for v in visual_shapes]
#	rgbaColors = [v.material_rgba for v in visual_shapes]
#	visualFramePositions = [v.origin_xyz for v in visual_shapes]
#	visualFrameOrientations = [p.getQuaternionFromEuler(v.origin_rpy) for v in visual_shapes]
		

#	if (len(shapeTypes)):
#		#print("len(shapeTypes)=",len(shapeTypes))
#		#print("len(halfExtents)=",len(halfExtents))
#		#print("len(radii)=",len(radii))
#		#print("len(lengths)=",len(lengths))
#		#print("len(fileNames)=",len(fileNames))
#		#print("len(meshScales)=",len(meshScales))
#		#print("len(rgbaColors)=",len(rgbaColors))
#		#print("len(visualFramePositions)=",len(visualFramePositions))
#		#print("len(visualFrameOrientations)=",len(visualFrameOrientations))

#		VisualShapeIndex = p.createVisualShapeArray(shapeTypes=shapeTypes,
#				halfExtents=halfExtents,
#				radii=radii,
#				lengths=lengths,
#				fileNames=fileNames,
#				meshScales=meshScales,
#				rgbaColors=rgbaColors,
#				visualFramePositions=visualFramePositions,
#				visualFrameOrientations=visualFrameOrientations,
#				physicsClientId=physicsClientId)

#	return VisualShapeIndex

#def createShapes(link : UrdfLink, physicsClientId=0):
#	return [createCollisionShape(link.urdf_visual_shapes, physicsClientId), 
#		createVisualShape(link.urdf_visual_shapes, physicsClientId)]
