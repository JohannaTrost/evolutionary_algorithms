from Creature import *

class evolution(object):
	def __init__(self, num_generations = 10, pop_size = 5):
		self.population = []

		self.num_generation = num_generations
		self.pop_size = pop_size


	def selection(self):
		pop = []
		for creature in self.population:
		    pop.append(list(creature.distFromStart(), creature))

		sortedPop = pop.sorted(key=lambda c:c[0])