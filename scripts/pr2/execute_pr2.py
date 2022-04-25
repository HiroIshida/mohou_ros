#!/usr/bin/env python3
import os
import argparse
from typing import Optional
import rospy
import rospkg
import numpy as np
from skrobot.models import PR2
from skrobot.model import Joint
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from mohou.types import AngleVector, GripperState

from mohou_ros_utils.config import Config
from mohou_ros_utils.executor import ExecutorBase
from pr2_controller_utils import check_pr2_is_executable


class SkrobotPR2Executor(ExecutorBase):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface

    def post_init_hook(self):
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = PR2ROSRobotInterface(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        check_pr2_is_executable()

    def send_command(self, av: AngleVector, gs: Optional[GripperState] = None) -> None:
        for angle, joint_name in zip(av.numpy(), self.control_joint_names):
            self.robot_model.__dict__[joint_name].joint_angle(angle)
        assert self.current_av is not None
        rospy.loginfo('current_av {}, target_av {}'.format(self.current_av.numpy(), av.numpy()))

        if not self.dryrun:
            self.robot_interface.angle_vector(
                self.robot_model.angle_vector(), time=1.0, time_scale=1.0)
            if gs is not None:
                # TOOD(HiroIshdia): handle gs is multi
                self.robot_interface.move_gripper('rarm', gs.numpy().item())

    def get_angle_vector(self) -> AngleVector:
        angles = []
        for joint_name in self.control_joint_names:
            self.robot_interface.update_robot_state(wait_until_update=True)
            joint: Joint = self.robot_interface.robot.__dict__[joint_name]
            angles.append(joint.joint_angle())
        return AngleVector(np.array(angles))


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))
    parser.add_argument('--force', action='store_true', help='disable dry option')

    args = parser.parse_args()
    config = Config.from_yaml_file(args.config)
    force = args.force

    rospy.init_node('executor', disable_signals=True)
    executor = SkrobotPR2Executor(config, dryrun=(not force))

    try:
        while(True):
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        rospy.loginfo('finish')
        executor.on_termination()
