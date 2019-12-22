from src.Creature import *
import pybullet as p
import random

class evolution(object):
	def __init__(self, num_generations=10, pop_size=5):
		self.population = []
		self.genomes = []
		self.num_generation = num_generations
		self.pop_size = pop_size


	def selection(self):
		pop = []
		numSurvivors = int(0.5 * len(pop))
		for creature in self.population:
			# fill list with tuples of distance and individual
		    pop.append(list(creature.distFromStart(), creature))
		sortedPop = pop.sorted(key=lambda c:c[0])

		sortedPop = [currTuple[1] for currTuple in sortedPop]

		# calcul of select and k and value of coeff
		# from "Concepts fondamentaux des algorithmes évolutionnistes"
		# by Jean-Baptiste Mouret
		coeff = 1.1
		k = coeff ** (numSurvivors + 1) - 1
		survivor_ids = list(np.round(numSurvivors - (numSurvivors / np.log(k + 1)) * np.log(k * np.random.rand(numSurvivors) + 1)))

		survivor_ids += survivor_ids  # to ensure population length

		parents = []
		for this_survivor_id in survivor_ids:
			# pair each survivor with one randomly chosen survivor from the difference-set
			# between the selected survivor and the others
			not_this_survivor_ids = np.setdiff1d(survivor_ids, this_survivor_id)
			not_this_survivor_id = int(random.choice(not_this_survivor_ids))
			parents.append((sortedPop[int(this_survivor_id)], sortedPop[not_this_survivor_id]))

		return parents

	def disableCollision(self):
		for idx, individual in enumerate(self.population[:-1]):  # from first to second last
			for other_individual in self.population[idx + 1:]:  # from next (relative to above) to end

				# pair all link indices and disable collision (num of joints = num of links)
				for joint in range(-1, p.getNumJoints(individual.editor.multiId)):  # all joints to ...
					for other_joint in range(-1, p.getNumJoints(other_individual.editor.multiId)):  # ... all other joints
						p.setCollisionFilterPair(individual.editor.multiId,
												 other_individual.editor.multiId,
												 joint,
												 other_joint,
												 enableCollision=0)
