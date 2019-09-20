import pybullet as p
import time
import pybullet_data

def createBox(halfExtents, position):
	boxID = p.createVisualShape(p.GEOM_BOX, halfExtents=halfExtents)
	colID = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)

	return p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=colID,
                      baseVisualShapeIndex=boxID,
                      basePosition=position,
                      useMaximalCoordinates=True)
	



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")




boxId = createBox([1,1,2], [0,0,1])





for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()