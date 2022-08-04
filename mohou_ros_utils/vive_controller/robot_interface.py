import numpy as np
from abc import abstractmethod, ABC
from typing import Optional, Tuple

from mohou_ros_utils.utils import CoordinateTransform
from skrobot.models import PR2
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore


class RobotInterfaceBase(ABC):

    @abstractmethod
    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        pass

    @abstractmethod
    def get_real_robot_joint_angles(self) -> np.ndarray:
        pass

    @abstractmethod
    def move_gripper(self, pos: float):
        pass


class ScikitRobotPR2Interface(RobotInterfaceBase):
    robot_interface: PR2ROSRobotInterface

    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        assert joint_angles.ndim == 1
        self.robot_interface.angle_vector(joint_angles, time=time, time_scale=1.0)

    def get_real_robot_joint_angles(self) -> np.ndarray:
        return self.robot_interface.angle_vector()  # type: ignore


class RoseusRobotInterface(RobotInterfaceBase):

    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        pass
