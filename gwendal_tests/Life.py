#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pybullet as p
import time
import numpy as np

def init_physics(mode):
    if mode == "flat":
        p.connect(p.GUI)
        plane = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, plane)
        useMaximalCoordinates = True




class Box (object):
    """Boxes are specific shapes used as sections/parts of an Individuum body."""

    def __init__(self, x_size=1, y_size=1, z_size=1, mass=1):
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.mass = mass

class Individuum (object):
    """Individuum is a class which describe virtual creatures attributs.
    We can use it in pybullet functions, in order to make 3D rendering and simulations of those creatures"""

    def __init__(self, id, sections = None, collision_shapes_IDs = None, sections_nb=0):
        self.id = id
        self.sections_nb = sections_nb
        self.sections = [] #Sections/Boxes list
        self.collision_shapes_IDs = [] #list of ID's for each collision shape
        self.add_box(self.sections_nb)  #init creature with the specified nb of sections/boxes.

    def add_box(self, nb_box):
        for i in range(nb_box):
            self.sections.append(Box())
            self.add_colShapes(self)
        self.sections_nb = len(self.sections)

    def add_colShapes(self, random_arg):
        currentColBoxId = p.createCollisionShape(p.GEOM_BOX,
                                          halfExtents=[self.sections[-1].x_size/2, self.sections[-1].y_size/2, self.sections[-1].z_size/2])
        print("Add_colbox_ID: "+str(currentColBoxId))
        self.collision_shapes_IDs.append(currentColBoxId)


# connect to physics server and start GUI
init_physics("flat")

##           EXEC/ ##
print("\n\n\n::EXECUTION::")
"produce a 8 boxes linear creature (snake-like)"
indiv1 = Individuum(id=0, sections_nb=4)    # init with 4 boxes
indiv1.add_box(4)                           # +4 boxes = 8

print(indiv1.sections[3].x_size)
print("Nb sections in individuum_1: "+str(indiv1.sections_nb))

print("collision shapes :")
print(indiv1.collision_shapes_IDs)
print("\n::END::\n\n")

## Pybullet part/

# iterate creation of collision shapes for boxes in 1 Individuum
"""
colBoxId1 = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[genome['sizes'][0]['sX'], genome['sizes'][0]['sY'], genome['sizes'][0]['sZ']])
colBoxId2 = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[genome['sizes'][1]['sX'], genome['sizes'][1]['sY'], genome['sizes'][1]['sZ']])
colBoxId3 = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[genome['sizes'][2]['sX'], genome['sizes'][2]['sY'], genome['sizes'][2]['sZ']])
"""
## /Pybullet part




##          /EXEC ##
