import subprocess
import uuid
from abc import ABC, abstractmethod
from pathlib import Path
from typing import ClassVar, Dict, List, Type, TypeVar

import numpy as np
import pybullet as pb
import rospy
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.model import Link, RobotModel
from skrobot.models.pr2 import PR2

from mohou_ros.srv import EuslispDirectCommand, EuslispDirectCommandResponse
from mohou_ros_utils.pr2.params import PR2LarmProperty, PR2RarmProperty
from mohou_ros_utils.utils import (
    CoordinateTransform,
    euslisp_unit_to_standard_unit,
    standard_unit_to_euslisp_unit,
)


class RobotControllerBase(ABC):
    robot_model: RobotModel

    @abstractmethod
    def update_real_robot(self, time: float) -> None:
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

    @property
    @abstractmethod
    def control_joint_names(self) -> List[str]:
        pass

    @property
    @abstractmethod
    def end_effector_name(self) -> str:
        pass

    def solve_inverse_kinematics(self, tf_gripper2base_target: CoordinateTransform) -> bool:
        joints = [self.robot_model.__dict__[jname] for jname in self.control_joint_names]
        link_list = [joint.child_link for joint in joints]
        end_effector = self.robot_model.__dict__[self.end_effector_name]
        av_next = self.robot_model.inverse_kinematics(
            tf_gripper2base_target.to_skrobot_coords(), end_effector, link_list, stop=5
        )
        is_solved = isinstance(av_next, np.ndarray)
        return is_solved

    def get_end_coords(self) -> CoordinateTransform:
        self.robot_model.angle_vector(self.get_real_robot_joint_angles())
        end_effector: Link = self.robot_model.__dict__[self.end_effector_name]
        coords = end_effector.copy_worldcoords()
        tf_gripperref2base = CoordinateTransform.from_skrobot_coords(coords, "gripper-ref", "base")
        return tf_gripperref2base


RobotControllerT = TypeVar("RobotControllerT", bound="RobotControllerBase")


class EuslispRobotController(RobotControllerBase):
    service_name: str
    proxy: rospy.ServiceProxy

    def eusserver_common_script(self, node_name: str, service_name: str) -> str:
        script = """
        (ros::load-ros-manifest "roseus")
        (ros::roseus-add-srvs "euslisp_command_srvs")
        (ros::roseus "{node_name}")
        {script_hook}
        (defun handle (req)
          (let* ((resp (send req :response))
                 (command-string (send req :command))
                 (command-expr (read-from-string command-string)))
            (print command-string)
            (eval command-expr)
            resp))
        (ros::advertise-service "{service_name}" euslisp_command_srvs::EuslispDirectCommand #'handle)
        (do-until-key
        (ros::spin-once)) """.format(
            script_hook=self.eus_script_hook(), node_name=node_name, service_name=service_name
        )
        return script

    def __init__(self):
        script_path = Path("/tmp/euslisp-{}.l".format(str(uuid.uuid4())))
        node_name = "roseus_command_server_{}".format(str(uuid.uuid4()).replace("-", ""))
        service_name = "roseus_command_{}".format(str(uuid.uuid4()).replace("-", ""))
        script = self.eusserver_common_script(node_name, service_name)
        with script_path.open(mode="w") as f:
            f.write(script)

        subprocess.Popen("roseus {}".format(str(script_path)), shell=True)
        assert script_path.exists()
        print("started eus server")

        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, EuslispDirectCommand)

        self.service_name = service_name
        self.proxy = proxy

    def update_real_robot(self, time: float) -> None:
        angle_list_stdunit = []
        for joint_name in self.eus_joint_name_list():
            angle = self.robot_model.__dict__[joint_name].joint_angle()
            angle_list_stdunit.append(angle)
        angle_vector_stdunit = np.array(angle_list_stdunit)
        angle_vector_eusunit = standard_unit_to_euslisp_unit(
            self.robot_model, self.eus_joint_name_list(), angle_vector_stdunit
        )

        float_vector_expr = "#f("
        for angle in angle_vector_eusunit:
            float_vector_expr += str(angle) + " "
        float_vector_expr += ")"

        time_ms = time * 1000.0
        script = """
        (send *ri* :angle-vector {} {})
        """.format(
            float_vector_expr, time_ms
        )
        self.proxy(script)

    def get_real_robot_joint_angles(self) -> np.ndarray:
        script = """
        (progn
          (let* (
                 (vec (send *ri* :state :potentio-vector))
                 (len (length vec))
                 (ret-string-list nil)
                 (ret-string nil)
                )
            (dotimes (i len)
              (push (string (aref vec i)) ret-string-list)
              (unless (eq i (- len 1))
                (push ", " ret-string-list))
            )
            (setq ret-string
              (reduce #'(lambda (a b) (concatenate string a b)) (nreverse ret-string-list)))
            (send resp :message ret-string)
            ))
        """
        ret: EuslispDirectCommandResponse = self.proxy(script)
        angle_vector_eusunit = np.array([float(e) for e in ret.message.split(",")])
        angle_vector_stdunit = euslisp_unit_to_standard_unit(
            self.robot_model, self.eus_joint_name_list(), angle_vector_eusunit
        )

        for joint_name, angle in zip(self.eus_joint_name_list(), angle_vector_stdunit):
            self.robot_model.__dict__[joint_name].joint_angle(angle)
        return self.robot_model.angle_vector()

    @abstractmethod
    def eus_script_hook(self) -> str:
        pass

    @abstractmethod
    def eus_joint_name_list(self) -> List[str]:
        pass


class EuslispPR2Controller(EuslispRobotController):
    def __init__(self):
        super().__init__()
        self.robot_model = PR2()

    def wait_interpolation(self) -> None:
        cmd = """(send *ri* :wait-interpolation)"""
        self.proxy(cmd)

    def eus_script_hook(self) -> str:
        script = """
        (load "package://pr2eus/pr2-interface.l")

        (defclass pr2-{arm_name}-interface
          :super pr2-interface
        )
        (defmethod pr2-{arm_name}-interface
          (:default-controller
           ()
           (append
            (send self :{arm_name}-controller)
           )))

        ;; the following script copied from pr2-init
        (unless (boundp '*pr2*) (pr2))
        (unless (boundp '*ri*) (setq *ri* (instance pr2-{arm_name}-interface :init)))
        (ros::spin-once)
        (send *ri* :spin-once)
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        (setq *robot* *pr2*)
        """.format(
            arm_name=self.arm_name()
        )
        return script

    def eus_joint_name_list(self) -> List[str]:
        return [
            "torso_lift_joint",
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint",
            "l_wrist_roll_joint",
            "r_shoulder_pan_joint",
            "r_shoulder_lift_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "r_forearm_roll_joint",
            "r_wrist_flex_joint",
            "r_wrist_roll_joint",
            "head_pan_joint",
            "head_tilt_joint",
        ]

    @abstractmethod
    def arm_name(self) -> str:
        pass


class EuslispPR2RarmController(PR2RarmProperty, EuslispPR2Controller):
    def move_gripper(self, pos: float) -> None:
        self.proxy("""(send *ri* :move-gripper :rarm {})""".format(pos))

    def arm_name(self) -> str:
        return "rarm"


class EuslispPR2LarmController(PR2LarmProperty, EuslispPR2Controller):
    def move_gripper(self, pos: float) -> None:
        self.proxy("""(send *ri* :move-gripper :larm {})""".format(pos))

    def arm_name(self) -> str:
        return "larm"


class SkrobotPybulletController(RobotControllerBase):
    pb_interface_id: int
    robot_id: int
    joint_name_to_id_table: Dict[str, int]
    is_realtime_joint: bool
    force: float = 200
    position_gain: float = 0.03
    velocity_gain: float = 1.0
    max_velocity: float = 1.0

    def __init__(
        self,
        robot_model: RobotModel,
        pb_robot_id: int,
        pb_interface_id: int,
        is_realtime_joint: bool,
    ):

        self.robot_model = robot_model
        self.pb_interface_id = pb_interface_id
        self.robot_id = pb_robot_id

        joint_table_all = {}
        for idx in range(pb.getNumJoints(self.robot_id)):
            joint_info = pb.getJointInfo(self.robot_id, idx)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("UTF-8")
            joint_table_all[joint_name] = joint_id
        self.joint_name_to_id_table = joint_table_all

        self.is_realtime_joint = is_realtime_joint

    def update_real_robot(self, time: float) -> None:
        # time is ignored here
        for joint_name in self.control_joint_names:
            assert joint_name in self.robot_model.__dict__, "{} is not in __dict__".format(
                joint_name
            )
            joint = self.robot_model.__dict__[joint_name]
            angle = joint.joint_angle()
            self._set_joint_angle(joint_name, angle, self.is_realtime_joint)

    def _set_joint_angle(self, joint_name: str, angle: float, real_time):
        joint_id = self.joint_name_to_id_table[joint_name]
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

    def update_real_robot(self, time: float) -> None:
        self.robot_interface.angle_vector(
            self.robot_model.angle_vector(), time=time, time_scale=1.0
        )

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


class SkrobotPR2RarmController(PR2RarmProperty, SkrobotPR2Controller):
    ri_type: ClassVar[Type[PR2ROSRobotInterface]] = RarmInterface

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("rarm", pos, effort=100)  # type: ignore


class SkrobotPR2LarmController(PR2LarmProperty, SkrobotPR2Controller):
    ri_type: ClassVar[Type[PR2ROSRobotInterface]] = LarmInterface

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("larm", pos, effort=100)  # type: ignore


class RoseusRobotInterface(RobotControllerBase):
    def update_real_robot(self, time: float) -> None:
        pass
