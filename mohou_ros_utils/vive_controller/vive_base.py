#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, ClassVar, Dict, Generic, List, Optional, Type, TypeVar

import genpy
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from sound_play.libsoundplay import SoundClient

from mohou_ros_utils.config import Config
from mohou_ros_utils.script_utils import (
    count_rosbag_file,
    create_rosbag_command,
    get_latest_rosbag_filename,
    get_rosbag_filepath,
)
from mohou_ros_utils.utils import CoordinateTransform, chain_transform
from mohou_ros_utils.vive_controller.robot_interface import RobotControllerT

MessageT = TypeVar("MessageT", bound=genpy.Message)


class TopicDataManager(ABC, Generic[MessageT]):
    name: str
    ttype: Type[MessageT]
    subscriber: rospy.Subscriber
    msg: Optional[MessageT] = None

    def __init__(self, name, ttype):
        # auto create subscriber
        self.name = name
        self.ttype = ttype
        self.subscriber = rospy.Subscriber(name, ttype, self.callback)  # type: ignore

    def callback(self, msg: MessageT):
        self.msg = msg


class PoseDataManager(TopicDataManager[PoseStamped]):
    processor: Optional[Callable] = None

    def __init__(self, name):
        super().__init__(name, PoseStamped)

    def register_processor(self, func: Callable):
        self.processor = func

    def process(self) -> None:
        if self.processor is None:
            return
        self.processor()


class JoyDataManager(TopicDataManager[Joy]):
    class Button(Enum):
        FRONT = 0
        BOTTOM = 1
        TOP = 2
        SIDE = 3

    trigger_times: List[Optional[float]]
    latest_process_times: List[Optional[float]]
    processors: List[Optional[Callable]]
    min_trigger_interval: float = 0.1
    min_process_interval: float = 0.3

    def __init__(self, name):
        super().__init__(name, Joy)
        self.trigger_times = [None for _ in range(4)]
        self.latest_process_times = [None for _ in range(4)]
        self.processors = [None for _ in range(4)]

    def callback(self, msg: Joy):
        t = msg.header.stamp.to_sec()
        for i in range(4):
            if msg.buttons[i] == 1:
                self.trigger_times[i] = t

    def is_recently_processed(self, button_type: Button) -> bool:
        latest_process_time = self.latest_process_times[button_type.value]
        if latest_process_time is None:
            return False
        current_time = rospy.Time.now().to_sec()
        return (current_time - latest_process_time) < self.min_process_interval

    def is_recently_triggered(self, button_type: Button) -> bool:
        trigger_time = self.trigger_times[button_type.value]
        if trigger_time is None:
            return False
        current_time = rospy.Time.now().to_sec()
        return (current_time - trigger_time) < self.min_trigger_interval

    def register_processor(self, button: Button, processor: Callable) -> None:
        self.processors[button.value] = processor

    def process(self) -> None:
        for button in self.Button:
            func = self.processors[button.value]
            if func is None:
                continue
            if not self.is_recently_triggered(button):
                continue
            if self.is_recently_processed(button):
                continue
            func()
            self.latest_process_times[button.value] = rospy.Time.now().to_sec()


class ViveController(ABC):
    joy_manager: JoyDataManager
    pose_manager: PoseDataManager
    scale: float
    timer_interval: float
    is_initialized: bool
    is_tracking: bool
    sound_client: SoundClient
    tf_handref2camera: Optional[CoordinateTransform] = None
    tf_gripperref2base: Optional[CoordinateTransform] = None

    def __init__(self, controller_id: str, scale: float):
        joy_topic_name = "/controller_{}/joy".format(controller_id)
        pose_topic_name = "/controller_{}_as_posestamped".format(controller_id)
        self.loginfo(
            "htc controller {} is assinged to {}".format(controller_id, self.__class__.__name__)
        )

        self.joy_manager = JoyDataManager(joy_topic_name)
        self.pose_manager = PoseDataManager(pose_topic_name)
        self.scale = scale
        self.timer_interval = 0.05
        self.is_initialized = False
        self.is_tracking = False
        self.sound_client = SoundClient(blocking=False)

        rospy.Timer(rospy.Duration(self.timer_interval), self.on_timer)

        # register
        self.pose_manager.register_processor(self.track_arm)
        self.joy_manager.register_processor(JoyDataManager.Button.TOP, self.on_and_off_tracker)

    def start(self) -> None:
        self.is_initialized = True
        rospy.loginfo("starting controller")

    def on_timer(self, event):
        t_start = time.time()

        if not self.is_initialized:
            return
        self.joy_manager.process()

        if self.is_tracking:
            self.pose_manager.process()

        t_elapsed_in_cb = time.time() - t_start
        process_time_rate = t_elapsed_in_cb / self.timer_interval
        if process_time_rate > 0.5:
            rospy.logwarn(
                "take {:.2f} sec ({:.1f} %) of callback duration {:.2f} sec".format(
                    t_elapsed_in_cb, process_time_rate * 100, self.timer_interval
                )
            )

    def track_arm(self) -> None:
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
        self.send_tracking_command(tf_gripper2base_target)

    def calibrate_controller(self) -> None:
        self.loginfo("calibrating controller")
        pose_msg = self.pose_manager.msg

        if pose_msg is None:
            rospy.logerr("no pose subscribed")
            return
        tf_handref2camera = CoordinateTransform.from_ros_pose(pose_msg.pose, "hand-ref", "camera")
        self.tf_handref2camera = tf_handref2camera
        self.tf_gripperref2base = self.get_robot_end_coords()
        assert self.tf_gripperref2base.src == "gripper-ref"
        assert self.tf_gripperref2base.dest == "base"

    def on_and_off_tracker(self) -> None:
        if self.is_tracking:
            self.is_tracking = False
            self.loginfo("turn off tracker")
        else:
            self.calibrate_controller()
            self.is_tracking = True
            self.loginfo("turn on tracker")

    def loginfo(self, message):
        rospy.loginfo("{} => ".format(self.log_prefix) + message)

    def logwarn(self, message):
        rospy.logwarn("{} => ".format(self.log_prefix) + message)

    @property
    @abstractmethod
    def log_prefix(self) -> str:
        pass

    @abstractmethod
    def send_tracking_command(self, tf_gripper2base_target: CoordinateTransform) -> None:
        pass

    @abstractmethod
    def get_robot_end_coords(self) -> CoordinateTransform:
        pass


class ViveRobotController(ViveController, Generic[RobotControllerT]):
    robot_con: RobotControllerT
    home_position_table: Dict[str, float]
    home_gripper_pos: float
    gripper_close: bool

    def __init__(
        self,
        controller_id: str,
        robot_con: RobotControllerT,
        scale: float,
        home_postion_table: Dict[str, float],
        home_gripper_pos: float,
    ):

        super().__init__(controller_id, scale)
        self.robot_con = robot_con
        self.gripper_close = False
        self.home_position_table = home_postion_table
        self.home_gripper_pos = home_gripper_pos

        self.joy_manager.register_processor(JoyDataManager.Button.BOTTOM, self.switch_grasp_state)
        self.joy_manager.register_processor(JoyDataManager.Button.SIDE, self.reset_to_home_position)
        self.loginfo("controller is initialized")

    def send_tracking_command(self, tf_gripper2base_target: CoordinateTransform) -> None:
        is_solved = self.robot_con.solve_inverse_kinematics(tf_gripper2base_target)
        if is_solved:
            self.robot_con.update_real_robot(time=0.8)
        else:
            self.logwarn("solving inverse kinematics failed")

    def get_robot_end_coords(self) -> CoordinateTransform:
        return self.robot_con.get_end_coords()

    def switch_grasp_state(self) -> None:
        if self.gripper_close:
            self.loginfo("gripper state CLOSE => OPEN")
            self.robot_con.move_gripper(0.06)
            self.gripper_close = False
        else:
            self.loginfo("gripper state OPEN => CLOSE")
            self.robot_con.move_gripper(0.0)
            self.gripper_close = True

    def reset_to_home_position(self, reset_grasp: bool = True) -> None:
        self.is_tracking = False
        self.loginfo("turn off tracker")
        self.loginfo("resetting to home position")

        for joint_name, angle in self.home_position_table.items():
            self.robot_con.robot_model.__dict__[joint_name].joint_angle(angle)

        self.robot_con.update_real_robot(3.0)

        if reset_grasp:
            self.robot_con.move_gripper(self.home_gripper_pos)
        self.robot_con.wait_interpolation()


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
        path = get_rosbag_filepath(self.config.project_path, time.strftime("%Y%m%d%H%M%S"))
        cmd = create_rosbag_command(path, self.config)
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
        n_count = count_rosbag_file(self.config.project_path) + 1  # because we are gonna add one
        self.sound_client.say("Finish saving rosbag. Total number is {}".format(n_count))

        self.closure_stop()
        self.closure_stop = None

    def switch_state(self) -> None:
        rospy.loginfo("switch rosbag state")
        if self.is_running:
            self.stop()
        else:
            self.start()


class RarmViveController(ViveRobotController[RobotControllerT]):
    rosbag_manager: RosbagManager
    log_prefix: ClassVar[str] = "Right"

    def __init__(
        self,
        robot_con: RobotControllerT,
        controller_id: str,
        scale: float,
        config: Config,
        gripper_joint_name: str = "r_gripper_joint",
    ):
        assert config.home_position is not None
        home_gripper_pos = config.home_position[gripper_joint_name]
        super().__init__(controller_id, robot_con, scale, config.home_position, home_gripper_pos)

        rosbag_manager = RosbagManager(config, self.sound_client)
        self.rosbag_manager = rosbag_manager
        self.joy_manager.register_processor(
            JoyDataManager.Button.FRONT, rosbag_manager.switch_state
        )


class LarmViveController(ViveRobotController[RobotControllerT]):
    project_path: Path
    log_prefix: ClassVar[str] = "Left"

    def __init__(
        self,
        robot_con: RobotControllerT,
        controller_id: str,
        scale: float,
        config: Config,
        gripper_joint_name: str = "l_gripper_joint",
    ):
        assert config.home_position is not None
        home_gripper_pos = config.home_position[gripper_joint_name]
        super().__init__(controller_id, robot_con, scale, config.home_position, home_gripper_pos)

        self.project_path = config.project_path
        self.joy_manager.register_processor(JoyDataManager.Button.FRONT, self.delete_latest_rosbag)

    def delete_latest_rosbag(self) -> None:
        latest_rosbag = get_latest_rosbag_filename(self.project_path)
        if latest_rosbag is None:
            message = "deleting rosbag failed because there is no rosbag"
            rospy.logwarn(message)
            self.sound_client.say(message)
        else:
            rospy.logwarn("delete rosbag file named {}".format(latest_rosbag))
            self.sound_client.say("delete latest rosbag")
            os.remove(latest_rosbag)
