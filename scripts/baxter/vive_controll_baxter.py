#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Optional

import rospkg
import rospy
from mohou.file import get_project_path
from skrobot.models.urdf import RobotModelFromURDF

from mohou_ros_utils.config import Config
from mohou_ros_utils.vive_controller.robot_interface import (
    SkrobotBaxteRarmController,
    SkrobotBaxterLarmController,
)
from mohou_ros_utils.vive_controller.utils import detect_controller_ids
from mohou_ros_utils.vive_controller.vive_base import (
    LarmViveController,
    RarmViveController,
)

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
        if len(controller_ids) == 2:
            print("controllers: {} are detected".format(controller_ids))
            break

    rospy.init_node("pr2_vive_mohou")

    rospack = rospkg.RosPack()
    model_urdf_path = Path(rospack.get_path("baxter_description"))
    baxter_urdf_path = model_urdf_path / "urdf" / "baxter.urdf"
    robot_model = RobotModelFromURDF(urdf_file=str(baxter_urdf_path))

    rarm_con = SkrobotBaxteRarmController(robot_model)
    larm_con = SkrobotBaxterLarmController(robot_model)

    rarm_vive_con = RarmViveController(rarm_con, controller_ids[0], scale, config)
    larm_vive_con = LarmViveController(larm_con, controller_ids[1], scale, config)
    rarm_vive_con.start()
    larm_vive_con.start()
    rospy.spin()
