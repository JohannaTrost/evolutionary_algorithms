import pybullet as p
import time
import numpy as np

def distance(x1, x2, y1, y2):
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

def createIndividuum(num_objects=3):

    #select size and power for each obj randomly
    gene_sizes = []
    gene_forces = []
    for obj in range(num_objects):
        sizeX, sizeY, sizeZ = np.random.rand(3) / 2 + 0.4
        velocity = (np.random.rand() * 2 - 1) * 10
        power = np.random.rand() * 5 + 5
        gene_sizes.append({'sX': sizeX, 'sY': sizeY, 'sZ': sizeZ})
        if obj < (num_objects - 1):
            gene_forces.append({'velo': velocity, 'power': power})

    genome = {'forces': gene_forces, 'sizes': gene_sizes}

    #create collision shapes for the objects
    colBoxId1 = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[genome['sizes'][0]['sX'], genome['sizes'][0]['sY'], genome['sizes'][0]['sZ']])
    colBoxId2 = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[genome['sizes'][1]['sX'], genome['sizes'][1]['sY'], genome['sizes'][1]['sZ']])
    colBoxId3 = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[genome['sizes'][2]['sX'], genome['sizes'][2]['sY'], genome['sizes'][2]['sZ']])

    mass = 1
    visualShapeId = -1

    # link obj 2 to obj 1 and obj 3 to obj 2
    link_Masses = [1, 1]
    linkCollisionShapeIndices = [colBoxId2, colBoxId3]
    linkVisualShapeIndices = [-1, -1]
    # set positions to connect objects
    linkPositions = [[0, 0, genome['sizes'][0]['sZ'] + genome['sizes'][1]['sZ']], [0, 0,  genome['sizes'][1]['sZ'] + genome['sizes'][2]['sZ']]]
    linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
    linkInertialFramePositions = [[0, 0, 0], [0, 0, 0]]
    linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
    indices = [0, 1]
    jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
    axis = [[0, 0, 1], [0, 0, 1]]

    basePosition=[0 ,0 ,4]
    # create object
    boxID = p.createMultiBody(mass,
                                    colBoxId1,
                                    visualShapeId,
                                    basePosition,
                                    baseOrientation=[0, 1, 0, 1],
                                    linkMasses=link_Masses,
                                    linkCollisionShapeIndices=linkCollisionShapeIndices,
                                      linkVisualShapeIndices=linkVisualShapeIndices,
                                      linkPositions=linkPositions,
                                      linkOrientations=linkOrientations,
                                      linkInertialFramePositions=linkInertialFramePositions,
                                      linkInertialFrameOrientations=linkInertialFrameOrientations,
                                      linkParentIndices=indices,
                                      linkJointTypes=jointTypes,
                                      linkJointAxis=axis)

    p.changeDynamics(boxID,
                   -1,
                   spinningFriction=0.001,
                   rollingFriction=0.001,
                   linearDamping=0.0)

    for joint in range(p.getNumJoints(boxID)):
        p.setJointMotorControl2(boxID, joint, p.VELOCITY_CONTROL, targetVelocity=genome['forces'][joint]['velo'], force=genome['forces'][joint]['power'])

    return boxID, genome, basePosition


def disable_collision(population):
    for idx, individual in enumerate(population[:-1]): # from first to second last
        for other_individual in population[idx + 1:]: # from next (relative to above) to end

            # pair all link indices and disable collision (num of joints = num of links)
            for joint in range(-1, p.getNumJoints(individual[0])): # all joints to ...
                for other_joint in range(-1, p.getNumJoints(other_individual[0])): # ... all other joints
                    p.setCollisionFilterPair(individual[0], other_individual[0], joint, other_joint, enableCollision=0)

sim_time = 10 #s
dt = 1. / 240.
p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
p.setGravity(0, 0, -9.81)

population = [createIndividuum() for i in range(10)]

disable_collision(population)

for i in range(int(sim_time / dt)):
    p.stepSimulation()
    time.sleep(dt)


distances = []
for individual in population:
    endPos = p.getBasePositionAndOrientation(individual[0])[0]
    basePosition = individual[2]
    distances.append(distance(endPos[0], basePosition[0], endPos[1], basePosition[1])) # distance(x1, x2, y1, y2)

num_generations = 3
generations = []
generations.append(population)

for idx in range(num_generations):

    # sort population along with distances and extract first element of each pair of zipped list
    indices_sorted = np.argsort(distances)

    distances_sorted = []
    for i ,individual in population:
        individual = population[indices_sorted[i]]
        distances_sorted.append(distances[indices_sorted[i]])
        print('index : {}'.format(i))

print('And the winner is individual {}'.format(np.argmax(distances))) # argmax is index of max value