from abc import ABC, abstractmethod
from typing import ClassVar, Dict, List, Type, TypeVar

import numpy as np
import pybullet as pb
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


RobotControllerT = TypeVar("RobotControllerT", bound="RobotControllerBase")


class SkrobotPybulletController(RobotControllerBase):
    pb_interface_id: int
    robot_id: int
    joint_name_to_id_table: Dict[str, int]
    force: float = 200
    position_gain: float = 0.1
    velocity_gain: float = 0.0
    max_velocity: float = 1.0

    @classmethod
    @abstractmethod
    def get_control_joint_names(cls) -> List[str]:
        pass

    def __init__(self, robot_model: RobotModel, pb_robot_id: int, pb_interface_id: int):
        self.pb_interface_id = pb_interface_id
        self.robot_id = pb_robot_id
        self.robot_model = robot_model

        # resolve joint_name => joint_id
        set([j.name for j in robot_model.joint_list])

        joint_table_all = {}
        for idx in range(pb.getNumJoints(self.robot_id)):
            joint_info = pb.getJointInfo(self.robot_id, idx)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("UTF-8")
            joint_table_all[joint_name] = joint_id
        self.joint_name_to_id_table = joint_table_all

    def update_real_robot(self, joint_angles: np.ndarray, time: float, real_time=False) -> None:
        """
        joint_angles: all joint angles
        """
        for joint_name in self.get_control_joint_names():
            assert joint_name in self.robot_model.__dict__, "{} is not in __dict__".format(joint_name)
            joint = self.robot_model.__dict__[joint_name]
            joint_id = self.joint_name_to_id_table[joint_name]
            angle = joint.joint_angle()
            if real_time:
                pb.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_id,
                    controlMode=pb.POSITION_CONTROL,
                    targetPosition=angle,
                    targetVelocity=0.0,
                    force=self.force,
                    positionGain=self.position_gain,
                    velocityGain=self.velocity_gain,
                    maxVelocity=self.max_velocity,
                )
            else:
                pb.resetJointState(self.robot_id, joint_id, angle)

    def get_real_robot_joint_angles(self) -> np.ndarray:
        angles = []
        for joint in self.robot_model.joint_list:
            joint_id = self.joint_name_to_id_table[joint.name]
            value = pb.getJointState(self.robot_id, joint_id)[0]
            angles.append(value)
        return np.array(angles)

    def move_gripper(self, pos: float):
        pass

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
