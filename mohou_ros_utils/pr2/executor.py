#!/usr/bin/env python3
from dataclasses import asdict, dataclass

import numpy as np
import rospy
from mohou.types import AngleVector, AnotherGripperState, ElementDict, GripperState
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.model import Joint, LinearJoint, RotationalJoint
from skrobot.models import PR2

from mohou_ros.msg import ControlCommand
from mohou_ros_utils.executor import ExecutorBase
from mohou_ros_utils.pr2.controller_utils import check_pr2_is_executable


class SkrobotPR2Executor(ExecutorBase):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface

    def _post_init(self):
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = PR2ROSRobotInterface(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        check_pr2_is_executable()

    def on_timer(self, event):
        if self.running:
            self.check_home_position_consistensy()
        super().on_timer(event)

    def send_command(self, edict_next: ElementDict, edict_current: ElementDict) -> None:

        if GripperState in edict_next:
            gs_current = edict_current[GripperState]
            gs_next = edict_next[GripperState]
            rospy.loginfo("current_gs {}, next_gs {}".format(gs_current.numpy(), gs_next.numpy()))

        if AnotherGripperState in edict_next:
            ags_current = edict_current[AnotherGripperState]
            ags_next = edict_next[AnotherGripperState]
            rospy.loginfo(
                "current_ags {}, next_ags {}".format(ags_current.numpy(), ags_next.numpy())
            )

        av_current = edict_current[AngleVector]
        av_next = edict_next[AngleVector]
        rospy.loginfo("current_av {}, next_av {}".format(av_current.numpy(), av_next.numpy()))

        for angle, joint_name in zip(av_next.numpy(), self.control_joint_names):
            self.robot_model.__dict__[joint_name].joint_angle(angle)

        if not self.dryrun:
            self.robot_interface.angle_vector(
                self.robot_model.angle_vector(), time=1.0, time_scale=1.0
            )

            if GripperState in edict_next:
                self.robot_interface.move_gripper("rarm", gs_next.numpy().item())  # type: ignore

            if AnotherGripperState in edict_next:
                self.robot_interface.move_gripper("larm", ags_next.numpy().item())  # type: ignore

    def get_angle_vector(self) -> AngleVector:
        angles = []
        for joint_name in self.control_joint_names:
            self.robot_interface.update_robot_state(wait_until_update=True)
            joint: Joint = self.robot_interface.robot.__dict__[joint_name]
            angles.append(joint.joint_angle())
        return AngleVector(np.array(angles))

    def check_home_position_consistensy(self) -> None:
        assert self.config.home_position is not None
        home_position_dict = self.config.home_position

        @dataclass
        class ErrorDetail:
            joint_name: str
            angle: float
            angle_desired: float
            error: float
            threshold: float

        error_detail_list = []

        for joint_name, angle_desired in home_position_dict.items():

            is_controlling_joint = joint_name in self.control_joint_names
            if is_controlling_joint:
                continue

            model_of_ri = self.robot_interface.robot
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


class EusPR2Executor(ExecutorBase):
    def _post_init(self):
        self.pub_command = rospy.Publisher(
            "/mohou_angle_vector_command", ControlCommand, queue_size=3
        )

    def send_command(self, edict_next: ElementDict, edict_current: ElementDict) -> None:

        if GripperState in edict_next:
            gs_current = edict_current[GripperState]
            gs_next = edict_next[GripperState]
            rospy.loginfo("current_gs {}, next_gs {}".format(gs_current.numpy(), gs_next.numpy()))

        if AnotherGripperState in edict_next:
            ags_current = edict_current[AnotherGripperState]
            ags_next = edict_next[AnotherGripperState]
            rospy.loginfo(
                "current_ags {}, next_ags {}".format(ags_current.numpy(), ags_next.numpy())
            )

        av_current = edict_current[AngleVector]
        av_next = edict_next[AngleVector]
        rospy.loginfo("current_av {}, next_av {}".format(av_current.numpy(), av_next.numpy()))

        control_cmd = ControlCommand()
        control_cmd.header.stamp = rospy.Time.now()

        assert len(av_next) == len(self.control_joint_names)
        control_cmd.joint_names = self.control_joint_names
        control_cmd.angles = list(av_next.numpy())

        if GripperState in edict_next:
            control_cmd.rarm_gripper_angle = gs_next.numpy()
        if AnotherGripperState in edict_next:
            control_cmd.larm_gripper_angle = ags_next.numpy()

        if not self.dryrun:
            self.pub_command.publish(control_cmd)

    def get_angle_vector(self) -> AngleVector:
        raise NotImplementedError("if you need this, please make a PR")
