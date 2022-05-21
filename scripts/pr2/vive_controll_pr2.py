#!/usr/bin/env python3
import argparse
from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, List, Optional, Type, Generic, TypeVar
import numpy as np

from skrobot.models import PR2
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore

import genpy
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.utils import chain_transform
from mohou_ros_utils.utils import CoordinateTransform
from mohou_ros_utils.pr2.params import rarm_joint_names
from mohou_ros_utils.config import Config


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
        if hasattr(self, 'callback'):
            self.subscriber = rospy.Subscriber(name, ttype, self.callback)  # type: ignore
        else:
            def cb(msg: MessageT):
                self.msg = msg
            self.subscriber = rospy.Subscriber(name, ttype, cb)


class PoseDataManager(TopicDataManager[PoseStamped]):
    process_time: Optional[float] = None
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
    process_times: List[Optional[float]]
    processors: List[Optional[Callable]]
    event_check_threshold: float = 0.1
    process_ignore_duration: float = 0.3

    def __init__(self, name):
        super().__init__(name, Joy)
        self.trigger_times = [None for _ in range(4)]
        self.process_times = [None for _ in range(4)]
        self.processors = [None for _ in range(4)]

    def callback(self, msg: Joy):
        t = msg.header.stamp.to_sec()
        for i in range(4):
            if msg.buttons[i] == 1:
                self.trigger_times[i] = t

    def is_processed(self, button_type: Button) -> bool:
        process_time = self.process_times[button_type.value]
        if process_time is None:
            return False
        current_time = rospy.Time.now().to_sec()
        return (current_time - process_time) < self.process_ignore_duration

    def is_triggered(self, button_type: Button) -> bool:
        trigger_time = self.trigger_times[button_type.value]
        if trigger_time is None:
            return False
        current_time = rospy.Time.now().to_sec()
        return (current_time - trigger_time) < self.event_check_threshold

    def register_processor(self, button: Button, processor: Callable) -> None:
        self.processors[button.value] = processor

    def process(self) -> None:
        for button in self.Button:
            func = self.processors[button.value]
            if func is None:
                continue
            if not self.is_triggered(button):
                continue
            if self.is_processed(button):
                continue
            func()
            self.process_times[button.value] = rospy.Time.now().to_sec()


class ViveController(ABC):
    right_joy_manager: JoyDataManager
    right_pose_manager: PoseDataManager
    is_initialized: bool
    is_tracking: bool

    def __init__(self, config: Config):
        self.right_joy_manager = JoyDataManager('/controller_LHR_FDF29FC7/joy')
        self.right_pose_manager = PoseDataManager('/controller_LHR_FDF29FC7_as_posestamped')

        rospy.Timer(rospy.Duration(0.1), self.on_timer)
        self.is_initialized = False
        self.is_tracking = False
        self.post_init_hook(config)

    @abstractmethod
    def post_init_hook(self, config: Config) -> None:
        pass

    def on_timer(self, event):
        if not self.is_initialized:
            return
        self.right_joy_manager.process()

        if self.is_tracking:
            self.right_pose_manager.process()


class PR2ViveController(ViveController):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface
    config: Config
    rarm_gripper_close: bool
    tf_handref2camera: Optional[CoordinateTransform]
    tf_gripperref2base: Optional[CoordinateTransform]

    def post_init_hook(self, config: Config) -> None:
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = PR2ROSRobotInterface(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        self.rarm_gripper_close = False

        self.right_joy_manager.register_processor(
            JoyDataManager.Button.BOTTOM, self.move_rarm_gripper)

        self.right_joy_manager.register_processor(
            JoyDataManager.Button.SIDE, self.reset_to_home_position)

        self.right_joy_manager.register_processor(
            JoyDataManager.Button.TOP, self.on_and_off_tracker)

        self.right_pose_manager.register_processor(self.track_rarm)
        self.config = config

        self.is_initialized = True
        self.is_tracking = False
        rospy.loginfo('controller is initialized')

    def track_rarm(self) -> None:
        pose_msg = self.right_pose_manager.msg

        is_ready = True
        if pose_msg is None:
            rospy.logwarn('no pose subscribed. stop tracking')
            is_ready = False

        if self.tf_handref2camera is None:
            rospy.logwarn('calibration is not done yet')
            is_ready = False

        if not is_ready:
            self.is_tracking = False
            rospy.logwarn('not ready for tracking.')
            self.is_tracking = False
            rospy.loginfo('turn off tracker')
            return

        assert pose_msg is not None
        assert self.tf_handref2camera is not None
        assert self.tf_gripperref2base is not None
        tf_hand2camera = CoordinateTransform.from_ros_pose(pose_msg.pose, 'hand', 'camera')
        tf_camera2handref = self.tf_handref2camera.inverse()
        tf_hand2handref = chain_transform(tf_hand2camera, tf_camera2handref)

        tf_gripper2gripperref = tf_hand2handref
        tf_gripper2gripperref.src = 'gripper'
        tf_gripper2gripperref.dest = 'gripper-ref'

        tf_gripper2base_target = chain_transform(tf_gripper2gripperref, self.tf_gripperref2base)

        joints = [self.robot_model.__dict__[jname] for jname in rarm_joint_names]
        link_list = [joint.child_link for joint in joints]

        av_next = self.robot_model.inverse_kinematics(
            tf_gripper2base_target.to_skrobot_coords(), self.robot_model.rarm_end_coords, link_list)

        if isinstance(av_next, np.ndarray):
            self.robot_interface.angle_vector(av_next, time=0.8, time_scale=1.0)
        else:
            rospy.logwarn('solving inverse kinematics failed')

    def move_rarm_gripper(self) -> None:
        if self.rarm_gripper_close:
            self.robot_interface.move_gripper('rarm', 0.06)  # type: ignore
            self.rarm_gripper_close = False
        else:
            self.robot_interface.move_gripper('rarm', 0.00)  # type: ignore
            self.rarm_gripper_close = True

    def on_and_off_tracker(self) -> None:
        if self.is_tracking:
            self.is_tracking = False
            rospy.loginfo('turn off tracker')
        else:
            self.calibrate_right_controller()
            self.is_tracking = True
            rospy.loginfo('turn on tracker')

    def calibrate_right_controller(self) -> None:
        rospy.loginfo('calibrating rarm controller')
        pose_msg = self.right_pose_manager.msg

        if pose_msg is None:
            rospy.logerr('no pose subscribed')
            return
        tf_handref2camera = CoordinateTransform.from_ros_pose(
            pose_msg.pose, 'hand-ref', 'camera')
        self.tf_handref2camera = tf_handref2camera

        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        coords = self.robot_model.rarm_end_coords
        self.tf_gripperref2base = CoordinateTransform.from_skrobot_coords(coords, 'gripper-ref', 'base')

    def reset_to_home_position(self) -> None:
        rospy.loginfo('resetting to home position')
        assert self.config.home_position is not None

        for joint_name in self.config.home_position.keys():
            angle = self.config.home_position[joint_name]
            self.robot_model.__dict__[joint_name].joint_angle(angle)
        self.robot_interface.angle_vector(self.robot_model.angle_vector(), time=3.0, time_scale=1.0)
        self.robot_interface.move_gripper('larm', self.config.home_position['l_gripper_joint'], effort=100)
        self.robot_interface.move_gripper('rarm', self.config.home_position['r_gripper_joint'], effort=100)
        self.robot_interface.wait_interpolation()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-pn', type=str, default=_default_project_name, help='project name')
    args = parser.parse_args()
    config = Config.from_project_name(args.pn)

    rospy.init_node('pr2_vive_mohou')
    cont = PR2ViveController(config)
    rospy.spin()
