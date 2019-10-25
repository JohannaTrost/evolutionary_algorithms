from urdfEditor import *

class Box(UrdfLink):
	def __init__(self, name, origin_xyz = [0,0,0], origin_rpy = [0,0,0], geom_extents = [1,1,1]):
		UrdfLink.__init__(self, name, 
					[UrdfVisual(origin_xyz, origin_rpy, name)], 
					[UrdfCollision(origin_xyz, origin_rpy, name)])

		self.urdf_collision_shapes[0].geom_type = p.GEOM_BOX
		self.urdf_collision_shapes[0].geom_extents = geom_extents
		
		self.urdf_visual_shapes[0].geom_type = p.GEOM_BOX
		self.urdf_visual_shapes[0].geom_extents = geom_extents


class Sphere(UrdfLink):
	def __init__(self, name, origin_xyz = [0,0,0], origin_rpy = [0,0,0], radius = 0.5):
		UrdfLink.__init__(self, name, 
					[UrdfVisual(origin_xyz, origin_rpy, name)], 
					[UrdfCollision(origin_xyz, origin_rpy, name)])

		self.urdf_collision_shapes[0].geom_type = p.GEOM_SPHERE
		self.urdf_collision_shapes[0].geom_radius = radius

		self.urdf_visual_shapes[0].geom_type = p.GEOM_SPHERE
		self.urdf_visual_shapes[0].geom_radius = radius
