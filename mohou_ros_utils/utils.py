import functools
import warnings
from dataclasses import asdict, dataclass
from typing import List, Optional

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2quaternion, quaternion2matrix, wxyz2xyzw

try:
    from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
except ModuleNotFoundError:
    ROSRobotInterfaceBase = None

from skrobot.model import Joint, LinearJoint, RobotModel, RotationalJoint

from mohou_ros_utils.config import Config


def deprecated(func):
    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter("always", DeprecationWarning)  # turn off filter
        warnings.warn(
            "Call to deprecated function {}.".format(func.__name__),
            category=DeprecationWarning,
            stacklevel=2,
        )
        warnings.simplefilter("default", DeprecationWarning)  # reset filter
        return func(*args, **kwargs)

    return new_func


@dataclass
class CoordinateTransform:
    trans: np.ndarray
    rot: np.ndarray
    src: Optional[str] = None
    dest: Optional[str] = None

    def __call__(self, vec_src: np.ndarray) -> np.ndarray:
        if vec_src.ndim == 1:
            return self.rot.dot(vec_src) + self.trans
        elif vec_src.ndim == 2:
            return self.rot.dot(vec_src.T).T + self.trans
        else:
            assert False

    def inverse(self) -> "CoordinateTransform":
        rot_new = self.rot.T
        trans_new = -rot_new.dot(self.trans)
        return CoordinateTransform(trans_new, rot_new, self.dest, self.src)

    @classmethod
    def from_ros_pose(cls, pose: Pose, src: Optional[str] = None, dest: Optional[str] = None):
        position = pose.position
        quat = pose.orientation
        trans = np.array([position.x, position.y, position.z])
        rot = quaternion2matrix([quat.w, quat.x, quat.y, quat.z])
        return cls(trans, rot, src, dest)

    def to_ros_pose(self) -> Pose:
        quat = wxyz2xyzw(matrix2quaternion(self.rot))
        pose = Pose(Point(*self.trans), Quaternion(*quat))
        return pose

    @classmethod
    def from_skrobot_coords(
        cls, coords: Coordinates, src: Optional[str] = None, dest: Optional[str] = None
    ):
        return cls(coords.worldpos(), coords.worldrot(), src, dest)

    def to_skrobot_coords(self) -> Coordinates:
        return Coordinates(self.trans, self.rot)


def chain_transform(
    tf_a2b: CoordinateTransform, tf_b2c: CoordinateTransform
) -> CoordinateTransform:
    if tf_a2b.dest is not None and tf_b2c.src is not None:
        assert tf_a2b.dest == tf_b2c.src, "{} does not match {}".format(tf_a2b.dest, tf_b2c.src)

    trans_a2c = tf_b2c.trans + tf_b2c.rot.dot(tf_a2b.trans)
    rot_a2c = tf_b2c.rot.dot(tf_a2b.rot)

    src_new = tf_a2b.src
    dest_new = tf_b2c.dest
    return CoordinateTransform(trans_a2c, rot_a2c, src_new, dest_new)


def standard_unit_to_euslisp_unit(
    robot: RobotModel, joint_names: List[str], joint_angles: np.ndarray
) -> np.ndarray:
    assert len(joint_names) == len(joint_angles)

    joint_angles_new = []
    for name, angle in zip(joint_names, joint_angles):
        joint = robot.__dict__[name]
        assert isinstance(joint, Joint)
        if isinstance(joint, RotationalJoint):
            joint_angles_new.append(np.rad2deg(angle))
        elif isinstance(joint, LinearJoint):
            joint_angles_new.append(angle * 1000.0)
        else:
            assert False, "{} is not compatible joint type".format(joint.__class__.__name__)
    return np.array(joint_angles_new)


def euslisp_unit_to_standard_unit(
    robot: RobotModel, joint_names: List[str], joint_angles_eus_unit: np.ndarray
) -> np.ndarray:
    assert len(joint_names) == len(joint_angles_eus_unit)

    joint_angles_new = []
    for name, angle in zip(joint_names, joint_angles_eus_unit):
        joint = robot.__dict__[name]
        assert isinstance(joint, Joint)
        if isinstance(joint, RotationalJoint):
            joint_angles_new.append(np.deg2rad(angle))
        elif isinstance(joint, LinearJoint):
            joint_angles_new.append(angle * 0.001)
        else:
            assert False, "{} is not compatible joint type".format(joint.__class__.__name__)
    return np.array(joint_angles_new)


def check_home_position_consistensy(
    ri: ROSRobotInterfaceBase, config: Config, exclude_keywords: List[str]
) -> None:
    assert config.home_position is not None
    home_position_dict = config.home_position

    @dataclass
    class ErrorDetail:
        joint_name: str
        angle: float
        angle_desired: float
        error: float
        threshold: float

    error_detail_list = []

    def is_skippable(joint_name: str) -> bool:
        # TODO: this is truly adhoc approach
        for kw in exclude_keywords:
            if kw in joint_name:
                return True
        return False

    for joint_name, angle_desired in home_position_dict.items():

        is_controlling_joint = joint_name in config.control_joints
        if is_controlling_joint:
            continue

        if is_skippable(joint_name):
            continue

        model_of_ri = ri.robot
        joint = model_of_ri.__dict__[joint_name]
        assert isinstance(joint, Joint)

        angle = joint.joint_angle()
        error = angle - angle_desired

        if isinstance(joint, RotationalJoint):
            threshold = np.deg2rad(2.0)  # 2.0deg
        elif isinstance(joint, LinearJoint):
            threshold = 0.01  # 1cm
        else:
            assert False

        if abs(error) > threshold:
            error_detail = ErrorDetail(joint_name, angle, angle_desired, error, threshold)
            error_detail_list.append(error_detail)

    if len(error_detail_list) > 0:
        n_error = len(error_detail_list)
        msg = "{} non-controlling joint angles are not consistent with home position.\n".format(
            n_error
        )
        for error_detail in error_detail_list:
            msg += str(asdict(error_detail)) + "\n"
        rospy.logerr(msg)
        assert False
