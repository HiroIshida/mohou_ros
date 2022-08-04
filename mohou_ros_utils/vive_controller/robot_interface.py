from abc import ABC, abstractmethod
from typing import Generic, Type, TypeVar

import numpy as np
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.models import PR2


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

    @abstractmethod
    def _post_init_setup(self, *arg, **kwargs):
        pass


PR2InterfaceT = TypeVar("PR2InterfaceT", bound=PR2ROSRobotInterface)


class ScikitRobotPR2Interface(RobotInterfaceBase, Generic[PR2InterfaceT]):
    robot_interface: PR2InterfaceT

    def _post_init_setup(self, interface_t: Type[PR2InterfaceT], robot_model: PR2):  # type: ignore
        self.robot_interface = interface_t(robot_model)

    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        assert joint_angles.ndim == 1
        self.robot_interface.angle_vector(joint_angles, time=time, time_scale=1.0)

    def get_real_robot_joint_angles(self) -> np.ndarray:
        return self.robot_interface.angle_vector()  # type: ignore


class RoseusRobotInterface(RobotInterfaceBase):
    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        pass
