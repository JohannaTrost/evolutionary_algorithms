import numpy as np
import jojo_evolution.simplePopulation_test as spt
import matplotlib.pylab as plt

num_obj = 3
generations, parent_ids, distances = spt.simulate_evolution(100, 100, num_obj)
col_names = ''

# generate column names
col_names += 'Generation, ID,'
for i in range(num_obj):
        col_names += 'x' + str(i+1) + ', '
        col_names += 'y' + str(i+1) + ', '
        col_names += 'z' + str(i + 1) + ', '
        col_names += 'power' + str(i + 1) + ', '
        col_names += 'velocity' + str(i + 1) + ', '
col_names += 'fitness(distance from start), parent1, parent2'

# fill table content into 2D array
data = []
for i, generation in enumerate(generations):
        for j, individual in enumerate(generation):
                row = []
                # generation
                row.append(float(i))
                # ID of individual
                row.append(float(individual[0]))
                # x,y,z,power,velocity for each box
                for box in individual[1]:
                        row = np.concatenate([np.asarray(row), np.asarray(box)])
                row = list(row)
                # fitness(distance)
                row.append(distances[i][j])
                # because no parents for first population
                if i > 0:
                        row.append(parent_ids[i][j][0])
                        row.append(parent_ids[i][j][1])
                else:
                        row.append(np.nan)
                        row.append(np.nan)
                data.append(row)
# save tale as csv
np.savetxt("jojo_evolution/evo_results.csv", data, delimiter=",", header=col_names, comments='')

# create plots over best and average performance of individuals
plt.figure()
plt.subplot(1, 2, 1)
plt.plot(np.mean(distances, axis=1))
plt.title('average per generation')
plt.ylabel('distance')
plt.xlabel('generation')
plt.subplot(1, 2, 2)
plt.plot(np.max(distances, axis=1))
plt.title('best performer per generation')
plt.xlabel('generation')
plt.suptitle('{} individuals'.format(len(distances[0])))
plt.savefig("jojo_evolution/evo_results.png")

