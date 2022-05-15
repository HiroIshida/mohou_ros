#!/usr/bin/env python3
import rospy
import numpy as np
from skrobot.models import PR2
from skrobot.model import Joint
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from mohou.types import AngleVector, GripperState, ElementDict

from mohou_ros_utils.executor import ExecutorBase
from mohou_ros_utils.pr2.controller_utils import check_pr2_is_executable


class SkrobotPR2Executor(ExecutorBase):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface

    def post_init_hook(self):
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = PR2ROSRobotInterface(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        check_pr2_is_executable()

    def send_command(self, edict_next: ElementDict, edict_current: ElementDict) -> None:

        if GripperState in edict_next:
            gs_current = edict_current[GripperState]
            gs_next = edict_next[GripperState]
            rospy.loginfo('current_gs {}, next_gs {}'.format(gs_current.numpy(), gs_next.numpy()))

        av_current = edict_current[AngleVector]
        av_next = edict_next[AngleVector]
        rospy.loginfo('current_av {}, next_av {}'.format(av_current.numpy(), av_next.numpy()))

        for angle, joint_name in zip(av_next.numpy(), self.control_joint_names):
            self.robot_model.__dict__[joint_name].joint_angle(angle)

        if not self.dryrun:
            self.robot_interface.angle_vector(
                self.robot_model.angle_vector(), time=1.0, time_scale=1.0)

            if GripperState in edict_next:
                # TOOD(HiroIshdia): handle gs is multi
                self.robot_interface.move_gripper('rarm', gs_next.numpy().item())  # type: ignore

    def get_angle_vector(self) -> AngleVector:
        angles = []
        for joint_name in self.control_joint_names:
            self.robot_interface.update_robot_state(wait_until_update=True)
            joint: Joint = self.robot_interface.robot.__dict__[joint_name]
            angles.append(joint.joint_angle())
        return AngleVector(np.array(angles))
