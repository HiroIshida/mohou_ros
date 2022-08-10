#!/usr/bin/env python3
import argparse
import time
from typing import Optional

import pybullet as pb
import pybullet_data
import rospy
from mohou.file import get_project_path
from skrobot.models import PR2

from mohou_ros_utils.config import Config
from mohou_ros_utils.pr2.params import PR2RarmProperty
from mohou_ros_utils.vive_controller.robot_interface import SkrobotPybulletController
from mohou_ros_utils.vive_controller.utils import detect_controller_ids
from mohou_ros_utils.vive_controller.vive_base import ViveRobotController


class SkrobotPybulletControllerPR2Rarm(PR2RarmProperty, SkrobotPybulletController):
    def __init__(self, pb_robot_id, pb_id):
        robot_model = PR2()
        robot_model.reset_manip_pose()
        super().__init__(robot_model, pb_robot_id=pb_robot_id, pb_interface_id=pb_id)


class BulletViveController(ViveRobotController[SkrobotPybulletControllerPR2Rarm]):
    @classmethod
    def create(cls, controller_id: str, config: Config, scale: float):
        # initialize pybullet world
        pb.connect(pb.GUI)  # or pybullet.DIRECT for non-graphical version
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pb.loadURDF("plane.urdf")
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
        pb.setGravity(0, 0, -10)
        robot_id = pb.loadURDF("/home/h-ishida/.skrobot/pr2_description/pr2.urdf")
        robot_con = SkrobotPybulletControllerPR2Rarm(robot_id, pb.GUI)
        robot_con.update_real_robot(1, real_time=False)
        return cls(controller_id, robot_con, config, scale)

    @property
    def log_prefix(self) -> str:
        return "pybullet"


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
    cont = BulletViveController.create(controller_id, config, 1.5)
    cont.start()
    time.sleep(100)
