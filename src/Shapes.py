from urdfEditor import *

class Box(UrdfLink):
	def __init__(self, name, origin=UrdfOrigin(), extents = [1,1,1]):
		UrdfLink.__init__(self, 
					name, 
					UrdfCollision(origin, UrdfBox(extents)), 
					UrdfVisual(origin, UrdfBox(extents)))


class Sphere(UrdfLink):
	def __init__(self, name, origin=UrdfOrigin(), radius = 0.5):
		UrdfLink.__init__(self, 
					name, 
					UrdfCollision(origin, UrdfSphere(radius)), 
					UrdfVisual(origin, UrdfSphere(radius)))
