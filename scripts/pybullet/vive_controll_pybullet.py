#!/usr/bin/env python3
import argparse
import time
from typing import List, Optional

import pybullet as pb
import pybullet_data
import rospy
from mohou.file import get_project_path
from skrobot.models import PR2

from mohou_ros_utils.config import Config
from mohou_ros_utils.pr2.params import rarm_joint_names
from mohou_ros_utils.vive_controller.robot_interface import SkrobotPybulletController
from mohou_ros_utils.vive_controller.utils import detect_controller_ids
from mohou_ros_utils.vive_controller.vive_base import ViveRobotController


class SkrobotPybulletControllerPR2Rarm(SkrobotPybulletController):
    def __init__(self, pb_robot_id, pb_id):
        robot_model = PR2()
        robot_model.reset_manip_pose()
        super().__init__(robot_model, pb_robot_id=pb_robot_id, pb_interface_id=pb_id)

    @classmethod
    def get_control_joint_names(cls) -> List[str]:
        return rarm_joint_names


class BulletViveController(ViveRobotController[SkrobotPybulletControllerPR2Rarm]):
    def __init__(self, controller_id: str, config: Config, scale: float):
        super().__init__(controller_id, scale)

        # initialize pybullet world
        pb.connect(pb.GUI)  # or pybullet.DIRECT for non-graphical version
        robot_id = pb.loadURDF("/home/h-ishida/.skrobot/pr2_description/pr2.urdf")
        robot_con = SkrobotPybulletControllerPR2Rarm(robot_id, pb.GUI)
        self.robot_con = robot_con
        self.robot_con.update_real_robot(1, real_time=False)
        self.config = config

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
        pb.setGravity(0, 0, -10)
        self.robot = pb.loadURDF("plane.urdf")
        # self.robot = pb.loadURDF("pr2_gripper.urdf")

    @property
    def log_prefix(self) -> str:
        return "pybullet"

    @property
    def arm_end_effector_name(self) -> str:
        return "r_gripper_tool_frame"

    @property
    def arm_joint_names(self) -> List[str]:
        return self.robot_con.get_control_joint_names()

    @property
    def gripper_joint_name(self) -> str:
        assert False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-scale", type=float, default=1.5, help="controller to real scaling")
    args, unknown = parser.parse_known_args()

    scale: float = args.scale
    project_name: Optional[str] = args.pn

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    while True:
        controller_ids = detect_controller_ids()
        print("waiting for vive controllers detected")
        if len(controller_ids) > 0:
            print("controllers: {} are detected".format(controller_ids))
            break
    controller_id = controller_ids[0]

    rospy.init_node("pybullet_vive_mohou")
    cont = BulletViveController(controller_id, config, 1.5)
    cont.start()
    time.sleep(100)
