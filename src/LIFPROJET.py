import pybullet as p
import pybullet_data
import time
import math
from Shapes import *
from Creature import *

import lxml.etree as etree

##########################################
gui = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE), p.createVisualShape(p.GEOM_PLANE))


c = Creature("Creature")
c.addLimb("Base", "Limb1", [0,0,0.5], [0,0,0], [0,0,0.5])

c.addLimb("Base", "Limb2", [0.5,0,0], [0,math.pi/2,0], [0,0,1], [1,1,2])
c.addLimb("Base", "Limb3", [-0.5,0,0], [0,-math.pi/2,0], [0,0,1], [1,1,2])

c.load([0,0,1], [0,0,0], True)


p.setRealTimeSimulation(1, physicsClientId=gui)

while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)
