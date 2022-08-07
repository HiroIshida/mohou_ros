from abc import ABC, abstractmethod
from typing import ClassVar, Type, TypeVar

import numpy as np
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.model import RobotModel


class RobotControllerBase(ABC):
    robot_model: RobotModel

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
    def wait_interpolation(self) -> None:
        pass


PR2InterfaceT = TypeVar("PR2InterfaceT", bound=PR2ROSRobotInterface)


class SkrobotPR2Controller(RobotControllerBase):
    ri_type: ClassVar[Type[PR2ROSRobotInterface]]
    robot_interface: PR2ROSRobotInterface

    def __init__(self, robot_model: RobotModel):
        self.robot_interface = self.ri_type(robot_model)
        robot_model.angle_vector(self.robot_interface.angle_vector())
        self.robot_model = robot_model

    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        assert joint_angles.ndim == 1
        self.robot_interface.angle_vector(joint_angles, time=time, time_scale=1.0)

    def get_real_robot_joint_angles(self) -> np.ndarray:
        return self.robot_interface.angle_vector()  # type: ignore

    def wait_interpolation(self) -> None:
        self.robot_interface.wait_interpolation()


class RarmInterface(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.rarm_controller, self.torso_controller, self.head_controller]


class LarmInterface(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.larm_controller, self.torso_controller, self.head_controller]


class SkrobotPR2RarmController(SkrobotPR2Controller):
    ri_type: ClassVar[Type[PR2ROSRobotInterface]] = RarmInterface

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("rarm", pos, effort=100)  # type: ignore


class SkrobotPR2LarmController(SkrobotPR2Controller):
    ri_type: ClassVar[Type[PR2ROSRobotInterface]] = LarmInterface

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("larm", pos, effort=100)  # type: ignore


class RoseusRobotInterface(RobotControllerBase):
    def update_real_robot(self, joint_angles: np.ndarray, time: float) -> None:
        pass
