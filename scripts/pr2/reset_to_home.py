#!/usr/bin/env python3
import argparse
from typing import Optional

import rospy
from mohou.file import get_project_path
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.models import PR2

from mohou_ros_utils.config import Config
from mohou_ros_utils.utils import check_home_position_consistensy

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    robot = PR2()
    ri = PR2ROSRobotInterface(robot)
    robot.angle_vector(ri.angle_vector())

    assert config.home_position is not None

    for joint_name in config.home_position.keys():
        angle = config.home_position[joint_name]
        robot.__dict__[joint_name].joint_angle(angle)
    ri.angle_vector(robot.angle_vector(), time=2.0, time_scale=1.0)
    ri.move_gripper("larm", config.home_position["l_gripper_joint"], effort=100)
    ri.move_gripper("rarm", config.home_position["r_gripper_joint"], effort=100)
    ri.wait_interpolation()

    exclude_keywords = ["gripper", "laser_tilt", "caster", "motor_screw"]
    check_home_position_consistensy(ri, config, exclude_keywords)
    rospy.loginfo("reset to home position successful")
    assert False
