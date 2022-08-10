#!/usr/bin/env python3
import threading
import time
from typing import Dict, List

import pybullet as pb
import pybullet_data
import rospy
from skrobot.model import RotationalJoint
from skrobot.models.urdf import RobotModelFromURDF

from mohou_ros_utils.vive_controller.robot_interface import SkrobotPybulletController
from mohou_ros_utils.vive_controller.utils import detect_controller_ids
from mohou_ros_utils.vive_controller.vive_base import ViveRobotController


class SkrobotPandaController(SkrobotPybulletController):
    def __init__(self, pb_robot_id, pb_id):
        robot_model = RobotModelFromURDF(
            urdf_file="/home/h-ishida/.local/lib/python3.8/site-packages/pybullet_data/franka_panda/panda.urdf"
        )
        super().__init__(
            robot_model, pb_robot_id=pb_robot_id, pb_interface_id=pb_id, is_realtime_joint=False
        )

    @property
    def control_joint_names(self) -> List[str]:
        lst = []
        for joint in self.robot_model.joint_list:
            if isinstance(joint, RotationalJoint):
                lst.append(joint.name)
        return lst

    @property
    def end_effector_name(self) -> str:
        return "panda_hand"

    def move_gripper(self, pos: float):
        assert pos > -1e-3 and pos < 0.08 + 1e-3
        name1 = "panda_finger_joint1"
        name2 = "panda_finger_joint2"
        self._set_joint_angle(name1, pos * 0.5, real_time=True)
        self._set_joint_angle(name2, pos * 0.5, real_time=True)


class BulletViveController(ViveRobotController[SkrobotPandaController]):
    def __init__(self, controller_id: str, scale: float, home_position_table: Dict[str, float]):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pb.loadURDF("plane.urdf")
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
        pb.setGravity(0, 0, -10)
        robot_id = pb.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        robot_con = SkrobotPandaController(robot_id, pb.GUI)

        super().__init__(controller_id, robot_con, scale, home_position_table, 0.0)
        self.reset_to_home_position()

        class SimulationLoop(threading.Thread):
            def run(self):
                while True:
                    time.sleep(0.003)
                    pb.stepSimulation()

        t = SimulationLoop()
        t.start()

    @property
    def log_prefix(self) -> str:
        return "pybullet"


if __name__ == "__main__":
    while True:
        controller_ids = detect_controller_ids()
        print("waiting for vive controllers detected")
        if len(controller_ids) > 0:
            print("controllers: {} are detected".format(controller_ids))
            break
    controller_id = controller_ids[0]

    home_position = {
        "panda_joint1": 0.0,
        "panda_joint2": 0.7,
        "panda_joint3": 0.0,
        "panda_joint4": -0.5,
        "panda_joint5": 0.0,
        "panda_joint6": 1.3,
        "panda_joint7": -0.8,
    }

    rospy.init_node("pybullet_vive_mohou")
    cont = BulletViveController(controller_id, 1.5, home_position)
    cont.start()
    time.sleep(100)
