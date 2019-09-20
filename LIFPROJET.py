import pybullet as p
import time
import pybullet_data

def createBox(boxHalfLength, boxHalfWidth, boxHalfHeight):
	boxID = p.createVisualShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
	colID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])

	return p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=colID,
                      baseVisualShapeIndex=boxID,
                      basePosition=[0, 0, 1],
                      useMaximalCoordinates=True)
	



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")




boxId = createBox(1,1,2)





for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()