import pybullet as p
import pybullet_data
import time
import math
from Shapes import *
from Creature import *

from typing import Iterable, Tuple, Callable

##########################################
gui = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE),
                  p.createVisualShape(p.GEOM_PLANE))


def creatureControl(angles, parent, child):
    force = 200
    jointNames = c.getJointNames(parent, child)
    jointIndices = c.editor.get_joint_indices(jointNames)

    for i, jointIndex in enumerate(jointIndices):
        p.setJointMotorControl2(c.editor.multiId, jointIndex, p.POSITION_CONTROL, targetPosition=angles[i], force=force,
                                maxVelocity=1)


m = MotorController([
    (0, math.pi / 3),
    (-math.pi / 3, 0),
    (0, -math.pi / 3),
    (math.pi / 3, 0)],
    creatureControl,
    "B1", "B2")

c = Creature("Creature", [0, 0, 3])

c.addLimb("", "B1", childOrigin=UrdfOrigin([0, 0, 0]))
c.addLimb("B1", "B2", UrdfOrigin([1, 0, 0]), UrdfOrigin([1, 0, 0]))
c.addLimb("B1", "B3", UrdfOrigin([-1, 0, 0]), UrdfOrigin([-1, 0, 0]))

c.load([0, 0, 0], True)

p.setRealTimeSimulation(1, physicsClientId=gui)

m.start()
while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
    m.update(c.getAngleBetween("B1", "B2"))

    p.stepSimulation(physicsClientId=gui)
    time.sleep(0.01)
