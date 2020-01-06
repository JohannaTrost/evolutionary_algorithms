import time
from math import pi
from typing import List, Iterable, Tuple, Any, Dict

import pybullet as p
from lxml import etree
from lxml.builder import E

URDF_FLOAT_PREC = 5


def gen_float(value: float):
    return f'{value:.{URDF_FLOAT_PREC}f}'


def gen_vec(vec: List[float]):
    return " ".join(map(lambda f: gen_float(f), vec))  # map(), just for kicks


class UrdfSerializable(object):
    def to_urdf(self) -> etree.ElementBase:
        raise NotImplementedError()


class UrdfOrigin(UrdfSerializable):
    def __init__(self, origin_xyz=[0, 0, 0], origin_rpy=[0, 0, 0]):
        self.origin_xyz = origin_xyz
        self.origin_rpy = origin_rpy

    def to_urdf(self):
        return E.origin(rpy=gen_vec(self.origin_rpy), xyz=gen_vec(self.origin_xyz))


class UrdfInertial(UrdfSerializable):
    def __init__(self, origin=UrdfOrigin()):
        self.origin = origin
        self.mass = 1
        self.inertia_xxyyzz = [0, 0, 0]

    def to_urdf(self):
        return E.inertial(
            self.origin.to_urdf(),
            E.mass(value=gen_float(self.mass)),
            E.inertia(
                ixx=gen_float(self.inertia_xxyyzz[0]),
                ixy="0",
                ixz="0",
                iyy=gen_float(self.inertia_xxyyzz[1]),
                iyz="0",
                izz=gen_float(self.inertia_xxyyzz[2])
            )
        )


class UrdfContact(object):
    def __init__(self, lateral_friction=1, rolling_friction=0, spinning_friction=0):
        self.lateral_friction = lateral_friction
        self.rolling_friction = rolling_friction
        self.spinning_friction = spinning_friction


class UrdfGeometry(UrdfSerializable):
    def to_urdf(self):
        return E.geometry()


class UrdfBox(UrdfGeometry):
    def __init__(self, extent=[1, 1, 1]):
        self.extent = extent

    def to_urdf(self):
        xml_box = super().to_urdf()
        xml_box.append(E.box(size=gen_vec(self.extent)))
        return xml_box


class UrdfSphere(UrdfGeometry):
    def __init__(self, radius=0.5):
        self.radius = radius

    def to_urdf(self):
        xml_sphere = super().to_urdf()
        xml_sphere.append(E.sphere(radius=gen_float(self.radius)))
        return xml_sphere


class UrdfMaterial(UrdfSerializable):
    def __init__(self, name="default", color_rgba=[1.0, 0.0, 0.0, 1.0]):
        self.name = name
        self.colorRGBA = color_rgba

    def to_urdf(self):
        return E.material(
            E.color(rgba=gen_vec(self.colorRGBA)),

            name=self.name
        )


class UrdfVisual(UrdfSerializable):
    def __init__(self, origin=UrdfOrigin(), geometry=UrdfBox(), material=UrdfMaterial()):
        self.origin = origin
        self.geometry = geometry
        self.material = material

    def to_urdf(self):
        return E.visual(
            self.origin.to_urdf(),
            self.geometry.to_urdf(),
            self.material.to_urdf()
        )


class UrdfCollision(UrdfSerializable):
    def __init__(self, origin=UrdfOrigin(), geometry=UrdfBox()):
        self.origin = origin
        self.geometry = geometry

    def to_urdf(self):
        return E.collision(
            self.origin.to_urdf(),
            self.geometry.to_urdf()
        )


class UrdfLink(UrdfSerializable):
    def __init__(self, name="dummy", collision=UrdfCollision(), visual=UrdfVisual(), inertial=UrdfInertial()):
        self.name = name
        self.collision = collision
        self.visual = visual
        self.inertial = inertial

    def to_urdf(self):
        return E.link(
            self.inertial.to_urdf(),
            self.visual.to_urdf(),
            self.collision.to_urdf(),

            name=self.name
        )


class UrdfJoint(UrdfSerializable):
    def __init__(self, parent_name, child_name, name="joint_dummy", origin=UrdfOrigin()):
        self.parent_name = parent_name
        self.child_name = child_name
        self.name = name
        self.origin = origin

    def to_urdf(self):
        return E.joint(
            E.parent(link=self.parent_name),
            E.child(link=self.child_name),
            E.dynamics(damping="1.0", friction="0.0001"),
            self.origin.to_urdf(),

            name=self.name
        )


class UrdfJointRevolute(UrdfJoint):
    def __init__(self, parent_name, child_name, name="joint_dummy", origin=UrdfOrigin(), axis_xyz=[0, 0, 1],
                 lower_limit=-pi, upper_limit=pi):
        super().__init__(parent_name, child_name, name, origin)

        self.axis_xyz = axis_xyz
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def to_urdf(self):
        xml_joint = super().to_urdf()

        xml_joint.set("type", "revolute")
        xml_joint.append(E.axis(xyz=gen_vec(self.axis_xyz)))
        xml_joint.append(E.limit(effort="1000.0", lower=gen_float(self.lower_limit), upper=gen_float(self.upper_limit),
                                 velocity="0.5"))

        return xml_joint


class UrdfJointContinuous(UrdfJoint):
    def __init__(self,
                 parent_name,
                 child_name,
                 name="joint_dummy",
                 origin=UrdfOrigin(),
                 axis_xyz=[0, 0, 1]):
        super().__init__(parent_name, child_name, name, origin)

        self.axis_xyz = axis_xyz

    def to_urdf(self):
        xml_joint = super().to_urdf()

        xml_joint.set("type", "continuous")
        xml_joint.append(E.axis(xyz=gen_vec(self.axis_xyz)))
        return xml_joint


def is_in_interval_array(data: Iterable[Any], intervals: Iterable[Tuple[Any, Any]]) -> bool:
    for x, interval in zip(data, intervals):
        if not (interval[0] <= x <= interval[1]):
            return False
    return True


def time_milli() -> int:
    """
    :return: The current timestamp in milliseconds.
    """
    return int(round(time.time() * 1000))


# A pair of float representing the timestamp in milliseconds and angle target.
MotorTargetPair = Tuple[float, float]
# A list of motor targets.
MotorDurations = List[MotorTargetPair]


class MotorData(object):
    jointName: str
    positionTargets: MotorDurations
    force: int

    def __init__(self, joint_name: str, targets: MotorDurations):
        self.jointName = joint_name
        self.positionTargets = targets

        self.force = 200

        self._currentIndex = 0

    @property
    def index(self) -> int:
        """
        :return: The current target index.
        """
        return self._currentIndex

    @index.setter
    def index(self, value: int) -> None:
        self._currentIndex = value

    @property
    def current_target(self) -> MotorTargetPair:
        """
        :return: The current target pair of the motor.
        """
        return self.positionTargets[self.index]

    def current_speed(self) -> float:
        if self.index <= 0:
            return self.current_target[1] / self.current_target[0] / 1000
        else:
            return (self.current_target[1] - self.positionTargets[self.index - 1][1]) / (
                    self.current_target[0] - self.positionTargets[self.index - 1][0]) / 1000


class UrdfEditor(object):
    links: List[UrdfLink]
    joints: List[UrdfJoint]
    linkNameToIndex: Dict[str, int]
    jointNameToIndex: Dict[str, int]
    motorControllers: Dict[str, MotorData]

    def __init__(self, robot_name="robot"):
        self.robotName = robot_name
        self.multiId = -1

        self.links = []
        self.joints = []
        self.linkNameToIndex = {}
        self.jointNameToIndex = {}
        self.motorControllers = {}

        self._motorStartTime = 0

    def save_urdf(self, file_name, save_visuals=True):
        root = E.robot(name=self.robotName)

        for link in self.links:
            root.append(link.to_urdf())
        for joint in self.joints:
            root.append(joint.to_urdf())

        with open(file_name, "wb+") as f:
            string = etree.tostring(root, xml_declaration=True, encoding="utf-8", pretty_print=True)
            f.write(string)

    def add_link(self, link: UrdfLink):
        self.linkNameToIndex[link.name] = len(self.links)
        self.links.append(link)

    def add_joint(self, joint: UrdfJoint):
        self.jointNameToIndex[joint.name] = len(self.joints)
        self.joints.append(joint)

    def add_motor_controller(self, joint_name: str, controller: MotorData):
        self.motorControllers[joint_name] = controller

    def get_link(self, link_name: str) -> UrdfLink:
        return self.links[self.linkNameToIndex[link_name]]

    def get_joint(self, joint_name: str) -> UrdfJoint:
        return self.joints[self.jointNameToIndex[joint_name]]

    def get_joint_position(self, name: str):
        return p.getJointState(self.multiId, self.jointNameToIndex[name])[0]

    def write_load(self, path_to_save: str, position=[0, 0, 0], orientation=[0, 0, 0], use_fixed_base=False):
        self.save_urdf(path_to_save)
        self.multiId = p.loadURDF(path_to_save, position, p.getQuaternionFromEuler(orientation),
                                  useFixedBase=use_fixed_base)

    def get_base_position(self):
        return p.getBasePositionAndOrientation(self.multiId)[0]

    def get_joint_indices(self, joint_names: Iterable[str]) -> List[int]:
        return [self.jointNameToIndex[k] for k in joint_names]

    def start(self):
        self._motorStartTime = time_milli()
        self.update()

    def update(self):
        for name, controller in self.motorControllers.items():
            if time_milli() > controller.current_target[0]:
                if controller.index + 1 >= len(controller.positionTargets):
                    controller.index = 0
                else:
                    controller.index += 1

                self._motorStartTime = time_milli()

                p.setJointMotorControl2(self.multiId, self.jointNameToIndex[controller.jointName], p.POSITION_CONTROL,
                                        targetPosition=controller.positionTargets[controller.index],
                                        targetVelocity=controller.current_speed(),
                                        force=controller.force)
