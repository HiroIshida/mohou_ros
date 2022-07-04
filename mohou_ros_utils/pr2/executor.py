#!/usr/bin/env python3
import numpy as np
import rospy
from mohou.types import AngleVector, AnotherGripperState, ElementDict, GripperState
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.model import Joint
from skrobot.models import PR2

from mohou_ros.msg import ControlCommand
from mohou_ros_utils.executor import ExecutorBase
from mohou_ros_utils.pr2.controller_utils import check_pr2_is_executable


class RarmInterface(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.rarm_controller]


class SkrobotPR2Executor(ExecutorBase):
    robot_model: PR2
    robot_interface: RarmInterface

    def _post_init(self):
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = RarmInterface(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        check_pr2_is_executable()

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
