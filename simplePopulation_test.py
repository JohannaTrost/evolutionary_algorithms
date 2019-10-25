import pybullet as p
import time
import numpy as np
import random
import math


def distance(x1, x2, y1, y2):
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5


def generate_rand_genome(num_objects):
    # select size and power for each obj randomly
    genome = []
    box_ids = []
    for obj in range(num_objects):
        # choose size power and velocity for current box randomly
        sizeX, sizeY, sizeZ = np.random.rand(3) / 2 + 0.4
        # number of power/ velocity has to correspont to number of liks (num_objects-1)
        velocity = (np.random.rand() * 2 - 1) * 10
        power = np.random.rand() * 5 + 5
        # for each box there is a list of (x,y,z) and 'force' -> [0: x, 1: y, 2: z, 3: velo, 4: pow]
        # that means genome[0] corresponds to first box etc.
        genome.append([sizeX, sizeY, sizeZ, velocity, power])

    return genome


def generate_visual_body(num_objs, genome):
    # declaration/init of parameters for multibody
    mass = 1
    visualShapeId = -1
    basePosition = [0, 0, 4]
    link_Masses = []  # [1, 1]
    linkCollisionShapeIndices = []  # [colBoxId2, colBoxId3]
    linkVisualShapeIndices = []  # [-1, -1]
    # set positions to connect objects
    linkPositions = []  # [[0, 0, genome['sizes'][0]['sZ'] + genome['sizes'][1]['sZ']],
    # [0, 0, genome['sizes'][1]['sZ'] + genome['sizes'][2]['sZ']]]
    linkOrientations = []  # [[0, 0, 0, 1], [0, 0, 0, 1]]
    linkInertialFramePositions = []  # [[0, 0, 0], [0, 0, 0]]
    linkInertialFrameOrientations = []  # [[0, 0, 0, 1], [0, 0, 0, 1]]
    indices = []  # [0, 1]
    jointTypes = []  # [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
    axis = []  # [[0, 0, 1], [0, 0, 1]]

    box_ids = []
    for obj in range(num_objs):

        # create collision shapes for the objects
        box_ids.append(p.createCollisionShape(p.GEOM_BOX,
                                              halfExtents=[genome[obj][0], genome[obj][1], genome[obj][2]]))  # [sizeX, sizeY, sizeZ]
        # set link and joint parameters for box (no link/joint for the first box)
        if (obj > 0):
            # at comments example for lists when 3 boxes in total
            link_Masses.append(1)  # [1, 1]
            linkCollisionShapeIndices.append(box_ids[obj])  # [colBoxId2, colBoxId3]
            linkVisualShapeIndices.append(-1)  # [-1, -1]
            # set positions to connect objects
            linkPositions.append([0, 0, genome[obj - 1][2] + genome[obj][
                2]])  # [[0, 0, sizeZ of 1. box + sizeZ of 2. box], [0, 0, sizeZ of 2. box + sizeZ of 3. box ]]
            linkOrientations.append([0, 0, 0, 1])  # [[0, 0, 0, 1], [0, 0, 0, 1]]
            linkInertialFramePositions.append([0, 0, 0])  # [[0, 0, 0], [0, 0, 0]]
            linkInertialFrameOrientations.append([0, 0, 0, 1])  # [[0, 0, 0, 1], [0, 0, 0, 1]]
            indices.append(obj - 1)  # [0, 1]
            jointTypes.append(p.JOINT_REVOLUTE)  # [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
            axis.append([0, 0, 1])  # [[0, 0, 1], [0, 0, 1]]

    # link obj 2 to obj 1 and obj 3 to obj 2
    # create object
    boxID = p.createMultiBody(mass,
                              box_ids[0],
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
        p.setJointMotorControl2(boxID,
                                joint,
                                p.VELOCITY_CONTROL,
                                targetVelocity=genome[joint][3],
                                force=genome[joint][4])
    return boxID, basePosition


def create_individual(num_objects=3, genome=[]):
    if not genome:
        # select size and power for each obj randomly
        genome = generate_rand_genome(num_objects)
    # create physical body of individual and get its ID and start position in world
    individual_id, base_position = generate_visual_body(num_objects, genome)

    return individual_id, genome, base_position


def disable_collision(pop):
    for idx, individual in enumerate(pop[:-1]): # from first to second last
        for other_individual in pop[idx + 1:]: # from next (relative to above) to end

            # pair all link indices and disable collision (num of joints = num of links)
            for joint in range(-1, p.getNumJoints(individual[0])): # all joints to ...
                for other_joint in range(-1, p.getNumJoints(other_individual[0])): # ... all other joints
                    p.setCollisionFilterPair(individual[0], other_individual[0], joint, other_joint, enableCollision=0)


# sort population along with distances
def sort(pop, dist):
    # list of indices of sorted distances
    indices_sorted = np.argsort(dist)[::-1][:len(dist)]
    # sort population and distances with help of indices_sorted
    pop_sorted = [pop[i] for i in indices_sorted]
    dist_sorted = [dist[i] for i in indices_sorted]

    return pop_sorted, dist_sorted


def selection(pop):
    survivors = []
    # want to keep 50% of the pop
    num_survivors = 0.5 * (len(pop))
    num_survivors = int(round(num_survivors))

    for i in range(num_survivors):
        # calcul of select and k and value of coeff
        # from "Concepts fondamentaux des algorithmes évolutionnistes"
        # by Jean-Baptiste Mouret
        coeff = 1.1
        k = coeff ** (num_survivors + 1) - 1
        select = num_survivors - (num_survivors / np.log(k + 1)) * np.log(k * np.random.rand() + 1)
        select = int(round(select))
        survivors.append(pop[select])
    # create list of parent couples for crossing
    non_selected = [i for i in range(num_survivors)]
    parents = []
    while non_selected:
        if len(non_selected) == 1:
            parents.append((survivors[non_selected[0]], survivors[np.random.randint(num_survivors)]))
            non_selected.remove(non_selected[0])
        elif len(non_selected) == 2:
            parents.append((survivors[non_selected[0]], survivors[non_selected[1]]))
            non_selected.remove(non_selected[0])
            non_selected.remove(non_selected[0])
        else:
            select_p1 = random.choice(non_selected)
            non_selected.remove(select_p1)
            select_p2 = random.choice(non_selected)
            non_selected.remove(select_p2)
            parents.append((survivors[select_p1], survivors[select_p2]))

    return parents


# define limit function
def limit(mid, diff, a):

    def shift_to_pos_bounds(lows, highs):
        neg_flags = lows < 0
        highs[neg_flags] = highs[neg_flags] + (-lows[neg_flags]) + 0.1
        lows[neg_flags] = 0.01

    limit_1 = np.asarray(mid) + np.asarray(diff) / 2 + a * np.asarray(diff)
    limit_2 = np.asarray(mid) - np.asarray(diff) / 2 - a * np.asarray(diff)
    # determine upper and lower bound
    limits_sorted = np.sort([limit_1, limit_2], axis=0)
    lower_bounds = np.array(limits_sorted[0])
    higher_bounds = np.array(limits_sorted[1])
    if True in lower_bounds[lower_bounds < 0]:
        shift_to_pos_bounds(lower_bounds, higher_bounds)
    return lower_bounds, higher_bounds


def randoms_between(lows, highs):
    lows = np.array(lows)
    highs = np.array(highs)
    # abs difference to shift random distribution that is natively between 0 and 1
    # you modify the range of values by multiplying and shift the lowest values by adding
    # Example: if you want random values between 1 and 3 you say: rand * 2 + 1, etc

    # array of random values between 0 and 1
    rand_values = np.random.rand(len(highs))  # len(high) == len(low)
    differences = highs - lows
    # random values between lower bound of limits and bound of limits + abs_difference
    rand_in_limits = (rand_values * differences) + lows

    return rand_in_limits


####### INPUT #######
# p1, p2 each is list of lists: [box1, box2,..., boxn]
# where each box consists of [x, y, z, velocity, power]
# a is alpha = 0.5
####### OUTPUT #######
# genome for individual [[x1, y1, z1, velo1, pow1][x2, y2, z2, velo2, pow2]...]
def generate_child_genome(p1, p2, a):
    child = []
    idx_obj = 0
    for obj_p1, obj_p2 in zip(p1, p2):
        # get o (average of parents)and d (distance between parents)
        o = np.mean([obj_p1, obj_p2], axis=0)
        d = np.asarray(obj_p1) - np.asarray(obj_p2)
        # compute limits in both directions
        # (calcul from p.39 of "Concepts fondamentaux des algorithmes évolutionnistes" by Jean-Baptiste Mouret)
        low_bounds, high_bounds = limit(o, d, a)
        child.append(randoms_between(low_bounds, high_bounds))
    return child


def crossing(parents, pop_size):
    next_generation = []
    alpha = 0.5;
    # choose parent couple until 2 individuals are left in the population to be the last 2 parents
    while len(next_generation) < pop_size:
        for couple in parents:
            print(couple)
            genome_child = generate_child_genome(couple[0][1], couple[1][1], alpha)
            next_generation.append(create_individual(genome=genome_child))  # contains [id, genome, basePosisiton]
    return next_generation

sim_time = 10 #s
dt = 1. / 240.
p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
p.setGravity(0, 0, -9.81)
assert(p.isConnected())

pop_size = 6;
initial_population = [create_individual() for i in range(pop_size)]
assert(len(initial_population) == pop_size and all(ind is not None for ind in initial_population))
disable_collision(initial_population)
num_generations = 3
generations = []
generations.append(initial_population)

curr_population = initial_population

for i in range(num_generations):
    for j in range(int(sim_time / dt)):
        p.stepSimulation()
        time.sleep(dt)
    distances = []
    for individual in curr_population:
        endPos = p.getBasePositionAndOrientation(individual[0])[0]
        basePosition = individual[2]
        distances.append(distance(endPos[0], basePosition[0], endPos[1], basePosition[1])) # distance(x1, x2, y1, y2)
    print('And the winner is individual {}'.format(np.argmax(distances)))  # argmax is index of max value
    curr_population, distances = sort(curr_population, distances)
    parents_selected = selection(curr_population)
    [p.removeBody(indiv[0]) for indiv in curr_population]
    curr_population = crossing(parents_selected, pop_size)
    disable_collision(curr_population)


