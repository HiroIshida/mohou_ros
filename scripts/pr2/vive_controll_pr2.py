#!/usr/bin/env python3
from abc import ABC, abstractmethod
import argparse
from dataclasses import dataclass
import time
import threading
import signal
import os
import subprocess
from enum import Enum
from typing import Callable, List, Optional, Type, Generic, TypeVar
import numpy as np

from skrobot.model import Link
from skrobot.models import PR2
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore

import genpy
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.utils import chain_transform
from mohou_ros_utils.utils import CoordinateTransform
from mohou_ros_utils.pr2.params import rarm_joint_names, larm_joint_names
from mohou_ros_utils.config import Config
from mohou_ros_utils.script_utils import create_rosbag_command


MessageT = TypeVar('MessageT', bound=genpy.Message)


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
    is_initialized: bool
    is_tracking: bool

    def __init__(self, config: Config, scale: float, joy_topic_name: str, pose_topic_name: str):
        self.joy_manager = JoyDataManager(joy_topic_name)
        self.pose_manager = PoseDataManager(pose_topic_name)
        self.scale = scale

        rospy.Timer(rospy.Duration(0.05), self.on_timer)
        self.is_initialized = False
        self.is_tracking = False
        self.post_init_hook(config)

    @abstractmethod
    def post_init_hook(self, config: Config) -> None:
        pass

    def on_timer(self, event):
        if not self.is_initialized:
            return
        self.joy_manager.process()

        if self.is_tracking:
            self.pose_manager.process()


class PR2ViveController(ViveController):
    robot_model: PR2
    robot_interface: Optional[PR2ROSRobotInterface]
    config: Config
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
        rospy.loginfo('{} => '.format(self.log_prefix) + message)

    def logwarn(self, message):
        rospy.logwarn('{} => '.format(self.log_prefix) + message)

    def post_init_hook(self, config: Config) -> None:

        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = self.robot_interface_type(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        self.gripper_close = False

        self.joy_manager.register_processor(
            JoyDataManager.Button.BOTTOM, self.switch_grasp_state)

        self.joy_manager.register_processor(
            JoyDataManager.Button.SIDE, self.reset_to_home_position)

        self.joy_manager.register_processor(
            JoyDataManager.Button.TOP, self.on_and_off_tracker)

        self.pose_manager.register_processor(self.track_arm)
        self.config = config

        self.is_initialized = True
        self.is_tracking = False
        self.loginfo('controller is initialized')

    def track_arm(self) -> None:
        assert self.robot_interface is not None

        pose_msg = self.pose_manager.msg

        is_ready = True
        if pose_msg is None:
            self.logwarn('no pose subscribed. stop tracking')
            is_ready = False

        if self.tf_handref2camera is None:
            self.logwarn('calibration is not done yet')
            is_ready = False

        if not is_ready:
            self.is_tracking = False
            self.logwarn('not ready for tracking.')
            self.is_tracking = False
            self.loginfo('turn off tracker')
            return

        assert pose_msg is not None
        assert self.tf_handref2camera is not None
        assert self.tf_gripperref2base is not None
        tf_hand2camera = CoordinateTransform.from_ros_pose(pose_msg.pose, 'hand', 'camera')
        tf_camera2handref = self.tf_handref2camera.inverse()
        tf_hand2handref = chain_transform(tf_hand2camera, tf_camera2handref)

        trans_scaled = tf_hand2handref.trans * self.scale
        tf_gripper2gripperref = CoordinateTransform(trans_scaled, tf_hand2handref.rot, 'gripper', 'gripper-ref')
        tf_gripper2base_target = chain_transform(tf_gripper2gripperref, self.tf_gripperref2base)

        joints = [self.robot_model.__dict__[jname] for jname in self.arm_joint_names]
        link_list = [joint.child_link for joint in joints]
        end_effector = self.robot_model.__dict__[self.arm_end_effector_name]
        av_next = self.robot_model.inverse_kinematics(
            tf_gripper2base_target.to_skrobot_coords(), end_effector, link_list, stop=5)

        if isinstance(av_next, np.ndarray):
            self.robot_interface.angle_vector(av_next, time=0.8, time_scale=1.0)
        else:
            self.logwarn('solving inverse kinematics failed')

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
            self.loginfo('turn off tracker')
        else:
            self.calibrate_controller()
            self.is_tracking = True
            self.loginfo('turn on tracker')

    def calibrate_controller(self) -> None:
        assert self.robot_interface is not None

        self.loginfo('calibrating controller')
        pose_msg = self.pose_manager.msg

        if pose_msg is None:
            rospy.logerr('no pose subscribed')
            return
        tf_handref2camera = CoordinateTransform.from_ros_pose(
            pose_msg.pose, 'hand-ref', 'camera')
        self.tf_handref2camera = tf_handref2camera

        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        end_effector: Link = self.robot_model.__dict__[self.arm_end_effector_name]
        coords = end_effector.copy_worldcoords()
        self.tf_gripperref2base = CoordinateTransform.from_skrobot_coords(coords, 'gripper-ref', 'base')

    def reset_to_home_position(self, reset_grasp: bool = True) -> None:
        assert self.robot_interface is not None

        self.is_tracking = False
        self.loginfo('turn off tracker')
        self.loginfo('resetting to home position')
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
    closure_stop: Optional[Callable] = None

    @property
    def is_running(self) -> bool:
        return self.closure_stop is not None

    def start(self):
        assert not self.is_running
        cmd = create_rosbag_command(self.config)
        p = subprocess.Popen(cmd)
        rospy.loginfo(p)
        share = {'is_running': True}

        def closure_stop():
            share['is_running'] = False

        self.closure_stop = closure_stop

        class Observer(threading.Thread):

            def run(self):
                while True:
                    time.sleep(0.5)
                    if not share['is_running']:
                        rospy.loginfo('kill rosbag process')
                        os.kill(p.pid, signal.SIGKILL)
                        break

        thread = Observer()
        thread.start()

    def stop(self):
        assert self.is_running
        assert self.closure_stop is not None
        self.closure_stop()
        self.closure_stop = None

    def switch_state(self):
        rospy.loginfo('switch rosbag state')
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
    arm_end_effector_name: str = 'r_gripper_tool_frame'
    gripper_joint_name: str = 'r_gripper_joint'
    log_prefix: str = 'Right'

    def __init__(self, config: Config, scale: float):
        super().__init__(config, scale, '/controller_LHR_FDF29FC7/joy', '/controller_LHR_FDF29FC7_as_posestamped')
        self.rosbag_manager = RosbagManager(config)

        self.joy_manager.register_processor(
            JoyDataManager.Button.FRONT, self.rosbag_manager.switch_state)

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper('rarm', pos, effort=100)  # type: ignore


class PR2LeftArmViveController(PR2ViveController):

    class LarmInterface(PR2ROSRobotInterface):
        def default_controller(self):
            return [self.larm_controller, self.torso_controller, self.head_controller]

    robot_interface_type = LarmInterface  # type: ignore
    arm_joint_names: List[str] = larm_joint_names
    arm_end_effector_name: str = 'l_gripper_tool_frame'
    gripper_joint_name: str = 'l_gripper_joint'
    log_prefix: str = 'Left'

    def __init__(self, config: Config, scale: float):
        super().__init__(config, scale, '/controller_LHR_FF3DFFC7/joy', '/controller_LHR_FF3DFFC7_as_posestamped')

    def move_gripper(self, pos: float) -> None:
        self.robot_interface.move_gripper('larm', pos, effort=100)  # type: ignore


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-pn', type=str, default=_default_project_name, help='project name')
    parser.add_argument('-scale', type=float, default=1.5, help='controller to real scaling')
    args, unknown = parser.parse_known_args()
    scale = args.scale
    config = Config.from_project_name(args.pn)

    rospy.init_node('pr2_vive_mohou')
    PR2RightArmViveController(config, scale)
    PR2LeftArmViveController(config, scale)
    rospy.spin()
