#!/usr/bin/env python3
import os
import argparse
import rospy
import rospkg
import numpy as np
from skrobot.models import PR2
from skrobot.model import Joint
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from mohou.types import AngleVector

from mohou_ros_utils.config import Config
from mohou_ros_utils.executor import ExecutorBase


class SkrobotPR2Executor(ExecutorBase):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface

    def post_init_hook(self):
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = PR2ROSRobotInterface(robot_model)

    def send_command(self, av: AngleVector) -> None:
        for angle, joint_name in zip(av.numpy(), self.control_joint_names):
            self.robot_model.__dict__[joint_name] = angle
        assert self.current_av is not None
        rospy.loginfo('current {}'.format(self.current_av.numpy()))
        rospy.loginfo('target {}'.format(av.numpy()))
        if not self.dryrun:
            self.robot_interface.angle_vector(
                self.robot_model.angle_vector, time=1.0, time_scale=1.0)

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

    args = parser.parse_args()
    config = Config.from_yaml_file(args.config)

    rospy.init_node('executor')
    executor = SkrobotPR2Executor(config)
    rospy.spin()
