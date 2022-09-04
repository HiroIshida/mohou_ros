#!/usr/bin/env python3
import argparse
from typing import Optional

from mohou.file import get_project_path
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.models import PR2

from mohou_ros_utils.config import Config

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("--rarm", action="store_true", help="loose rarm")
    parser.add_argument("--larm", action="store_true", help="loose larm")
    parser.add_argument("--mirror", action="store_true", help="mirror mode")
    parser.add_argument("--home", action="store_true", help="use home position")
    parser.add_argument("-open", type=float, default=0.06, help="max gripper position")
    parser.add_argument("-close", type=float, default=0.00, help="max gripper position")

    args = parser.parse_args()
    loose_larm: str = args.larm
    loose_rarm: str = args.rarm
    pos_open: float = args.open
    pos_close: float = args.close
    mirror: bool = args.mirror
    home: bool = args.home
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
