#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import threading
import time
from abc import abstractmethod
from dataclasses import dataclass
from typing import Callable, List, Optional, Type

import numpy as np
import rospy
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.model import Link
from skrobot.models import PR2
from sound_play.libsoundplay import SoundClient

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.pr2.params import larm_joint_names, rarm_joint_names
from mohou_ros_utils.script_utils import (
    count_rosbag_file,
    create_rosbag_command,
    get_latest_rosbag_filename,
    get_rosbag_filename,
)
from mohou_ros_utils.utils import CoordinateTransform, chain_transform
from mohou_ros_utils.vive_controller import JoyDataManager, ViveController


class PR2ViveController(ViveController):
    robot_model: PR2
    robot_interface: Optional[PR2ROSRobotInterface]
    config: Config
    sound_client: SoundClient
    gripper_close: bool
    tf_handref2camera: Optional[CoordinateTransform]
    tf_gripperref2base: Optional[CoordinateTransform]

    @property
    @abstractmethod
    def robot_interface_type(self) -> Type[PR2ROSRobotInterface]:
        pass

    @property
    @abstractmethod
    def arm_joint_names(self) -> List[str]:
        pass

    @property
    @abstractmethod
    def arm_end_effector_name(self) -> str:
        pass

    @property
    @abstractmethod
    def log_prefix(self) -> str:
        pass

    @property
    @abstractmethod
    def gripper_joint_name(self) -> str:
        pass

    @abstractmethod
    def move_gripper(self, pos: float) -> None:
        pass

    def loginfo(self, message):
        rospy.loginfo("{} => ".format(self.log_prefix) + message)

    def logwarn(self, message):
        rospy.logwarn("{} => ".format(self.log_prefix) + message)

    def post_init_hook(self) -> None:
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = self.robot_interface_type(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        self.gripper_close = False

        self.joy_manager.register_processor(JoyDataManager.Button.BOTTOM, self.switch_grasp_state)

        self.joy_manager.register_processor(JoyDataManager.Button.SIDE, self.reset_to_home_position)

        self.joy_manager.register_processor(JoyDataManager.Button.TOP, self.on_and_off_tracker)

        self.pose_manager.register_processor(self.track_arm)
        self.config = config
        self.sound_client = SoundClient(blocking=False)

        self.is_initialized = True
        self.is_tracking = False
        self.loginfo("controller is initialized")

    def track_arm(self) -> None:
        assert self.robot_interface is not None

        pose_msg = self.pose_manager.msg

        is_ready = True
        if pose_msg is None:
            self.logwarn("no pose subscribed. stop tracking")
            is_ready = False

        if self.tf_handref2camera is None:
            self.logwarn("calibration is not done yet")
            is_ready = False

        if not is_ready:
            self.is_tracking = False
            self.logwarn("not ready for tracking.")
            self.is_tracking = False
            self.loginfo("turn off tracker")
            return

        assert pose_msg is not None
        assert self.tf_handref2camera is not None
        assert self.tf_gripperref2base is not None
        tf_hand2camera = CoordinateTransform.from_ros_pose(pose_msg.pose, "hand", "camera")
        tf_camera2handref = self.tf_handref2camera.inverse()
        tf_hand2handref = chain_transform(tf_hand2camera, tf_camera2handref)

        trans_scaled = tf_hand2handref.trans * self.scale
        tf_gripper2gripperref = CoordinateTransform(
            trans_scaled, tf_hand2handref.rot, "gripper", "gripper-ref"
        )
        tf_gripper2base_target = chain_transform(tf_gripper2gripperref, self.tf_gripperref2base)

        joints = [self.robot_model.__dict__[jname] for jname in self.arm_joint_names]
        link_list = [joint.child_link for joint in joints]
        end_effector = self.robot_model.__dict__[self.arm_end_effector_name]
        av_next = self.robot_model.inverse_kinematics(
            tf_gripper2base_target.to_skrobot_coords(), end_effector, link_list, stop=5
        )

        if isinstance(av_next, np.ndarray):
            self.robot_interface.angle_vector(av_next, time=0.8, time_scale=1.0)
        else:
            self.logwarn("solving inverse kinematics failed")

    def switch_grasp_state(self) -> None:
        if self.gripper_close:
            self.move_gripper(0.06)
            self.gripper_close = False
        else:
            self.move_gripper(0.0)
            self.gripper_close = True

    def on_and_off_tracker(self) -> None:
        if self.is_tracking:
            self.is_tracking = False
            self.loginfo("turn off tracker")
        else:
            self.calibrate_controller()
            self.is_tracking = True
            self.loginfo("turn on tracker")

    def calibrate_controller(self) -> None:
        assert self.robot_interface is not None

        self.loginfo("calibrating controller")
        pose_msg = self.pose_manager.msg

        if pose_msg is None:
            rospy.logerr("no pose subscribed")
            return
        tf_handref2camera = CoordinateTransform.from_ros_pose(pose_msg.pose, "hand-ref", "camera")
        self.tf_handref2camera = tf_handref2camera

        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        end_effector: Link = self.robot_model.__dict__[self.arm_end_effector_name]
        coords = end_effector.copy_worldcoords()
        self.tf_gripperref2base = CoordinateTransform.from_skrobot_coords(
            coords, "gripper-ref", "base"
        )

    def reset_to_home_position(self, reset_grasp: bool = True) -> None:
        assert self.robot_interface is not None

        self.is_tracking = False
        self.loginfo("turn off tracker")
        self.loginfo("resetting to home position")
        assert self.config.home_position is not None

        for joint_name in self.config.home_position.keys():
            angle = self.config.home_position[joint_name]
            self.robot_model.__dict__[joint_name].joint_angle(angle)
        self.robot_interface.angle_vector(self.robot_model.angle_vector(), time=3.0, time_scale=1.0)
        self.config.home_position[self.gripper_joint_name]
        if reset_grasp:
            self.move_gripper(self.config.home_position[self.gripper_joint_name])
        self.robot_interface.wait_interpolation()


@dataclass
class RosbagManager:
    config: Config
    sound_client: SoundClient
    closure_stop: Optional[Callable] = None

    @property
    def is_running(self) -> bool:
        return self.closure_stop is not None

    def start(self) -> None:
        assert not self.is_running
        filename = get_rosbag_filename(self.config, time.strftime("%Y%m%d%H%M%S"))
        cmd = create_rosbag_command(filename, self.config)
        p = subprocess.Popen(cmd)
        rospy.loginfo(p)
        share = {"is_running": True}

        def closure_stop():
            share["is_running"] = False

        self.closure_stop = closure_stop

        class Observer(threading.Thread):
            def run(self):
                while True:
                    time.sleep(0.5)
                    if not share["is_running"]:
                        rospy.loginfo("kill rosbag process")
                        os.kill(p.pid, signal.SIGTERM)
                        break

        self.sound_client.say("Start saving rosbag")
        thread = Observer()
        thread.start()

    def stop(self) -> None:
        assert self.is_running
        assert self.closure_stop is not None
        n_count = count_rosbag_file(self.config) + 1  # because we are gonna add one
        self.sound_client.say("Finish saving rosbag. Total number is {}".format(n_count))

        self.closure_stop()
        self.closure_stop = None

    def switch_state(self) -> None:
        rospy.loginfo("switch rosbag state")
        if self.is_running:
            self.stop()
        else:
            self.start()


class PR2RightArmViveController(PR2ViveController):
    rosbag_manager: RosbagManager

    class RarmInterface(PR2ROSRobotInterface):
        def default_controller(self):
            return [self.rarm_controller, self.torso_controller, self.head_controller]

    robot_interface_type = RarmInterface  # type: ignore
    arm_joint_names: List[str] = rarm_joint_names
    arm_end_effector_name: str = "r_gripper_tool_frame"
    gripper_joint_name: str = "r_gripper_joint"
    log_prefix: str = "Right"

    def __init__(self, config: Config, scale: float):
        controller_id = "LHR_F7AFBF47"
        super().__init__(
            config,
            scale,
            "/controller_{}/joy".format(controller_id),
            "/controller_{}_as_posestamped".format(controller_id),
        )
        self.rosbag_manager = RosbagManager(config, self.sound_client)

        self.joy_manager.register_processor(
            JoyDataManager.Button.FRONT, self.rosbag_manager.switch_state
        )

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("rarm", pos, effort=100)  # type: ignore


class PR2LeftArmViveController(PR2ViveController):
    class LarmInterface(PR2ROSRobotInterface):
        def default_controller(self):
            return [self.larm_controller, self.torso_controller, self.head_controller]

    robot_interface_type = LarmInterface  # type: ignore
    arm_joint_names: List[str] = larm_joint_names
    arm_end_effector_name: str = "l_gripper_tool_frame"
    gripper_joint_name: str = "l_gripper_joint"
    log_prefix: str = "Left"

    def __init__(self, config: Config, scale: float):
        controller_id = "LHR_FD35BD42"
        super().__init__(
            config,
            scale,
            "/controller_{}/joy".format(controller_id),
            "/controller_{}_as_posestamped".format(controller_id),
        )

        self.joy_manager.register_processor(JoyDataManager.Button.FRONT, self.delete_latest_rosbag)

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper("larm", pos, effort=100)  # type: ignore

    def delete_latest_rosbag(self) -> None:
        latest_rosbag = get_latest_rosbag_filename(self.config)
        if latest_rosbag is None:
            message = "deleting rosbag failed because there is no rosbag"
            rospy.logwarn(message)
            self.sound_client.say(message)
        else:
            rospy.logwarn("delete rosbag file named {}".format(latest_rosbag))
            self.sound_client.say("delete latest rosbag")
            os.remove(latest_rosbag)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    parser.add_argument("-scale", type=float, default=1.5, help="controller to real scaling")
    args, unknown = parser.parse_known_args()
    scale = args.scale
    config = Config.from_project_name(args.pn)

    rospy.init_node("pr2_vive_mohou")
    PR2RightArmViveController(config, scale)
    PR2LeftArmViveController(config, scale)
    rospy.spin()
