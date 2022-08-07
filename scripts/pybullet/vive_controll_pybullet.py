#!/usr/bin/env python3
import time

import numpy as np
import pybullet as pb
import pybullet_data
import rospy
from skrobot.coordinates.math import (
    matrix2quaternion,
    quaternion2matrix,
    wxyz2xyzw,
    xyzw2wxyz,
)

from mohou_ros_utils.utils import CoordinateTransform
from mohou_ros_utils.vive_controller.vive_base import ViveController


class BulletViveController(ViveController):
    robot: int

    def __init__(self, scale: float = 1.5):
        controller_id = "LHR_F7AFBF47"
        joy_topic = "/controller_{}/joy".format(controller_id)
        pose_topic = "/controller_{}_as_posestamped".format(controller_id)
        super().__init__(joy_topic, pose_topic, scale)

        # initialize pybullet world
        pb.connect(pb.GUI)  # or pybullet.DIRECT for non-graphical version
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
        pb.setGravity(0, 0, -10)
        pb.loadURDF("plane.urdf")
        # self.robot = pb.loadURDF("pr2_gripper.urdf")
        self.robot = pb.loadURDF("./franka_panda/panda.urdf")

    @property
    def log_prefix(self) -> str:
        return "pybullet"

    def send_tracking_command(self, tf_gripper2base_target: CoordinateTransform) -> None:
        tf_gripper2base_target.trans
        mat = tf_gripper2base_target.rot
        wxyz2xyzw(matrix2quaternion(mat))
        # pb.resetBasePositionAndOrientation(self.robot, pos, quat)

    def get_robot_end_coords(self) -> CoordinateTransform:
        pos, quat = pb.getBasePositionAndOrientation(self.robot)
        mat = quaternion2matrix(xyzw2wxyz(quat))
        return CoordinateTransform(np.array(pos), mat, src="gripper-ref", dest="base")


rospy.init_node("pybullet_controller")
cont = BulletViveController()
cont.start()
time.sleep(100)
