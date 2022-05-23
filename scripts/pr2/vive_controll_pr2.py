#!/usr/bin/env python3
import argparse
from abc import ABC, abstractmethod
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
    joy_manager: JoyDataManager
    pose_manager: PoseDataManager
    scale: float
    is_initialized: bool
    is_tracking: bool

    def __init__(self, config: Config, scale: float, joy_topic_name: str, pose_topic_name: str):
        self.joy_manager = JoyDataManager(joy_topic_name)
        self.pose_manager = PoseDataManager(pose_topic_name)
        self.scale = scale

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
        self.joy_manager.process()

        if self.is_tracking:
            self.pose_manager.process()


class PR2ViveController(ViveController):
    robot_model: PR2
    robot_interface: PR2ROSRobotInterface
    config: Config
    gripper_close: bool
    tf_handref2camera: Optional[CoordinateTransform]
    tf_gripperref2base: Optional[CoordinateTransform]

    # set this
    robot_interface_type: Type[PR2ROSRobotInterface]
    arm_joint_names: List[str]
    arm_end_effector_name: str
    arm_name: str

    def loginfo(self, message):
        prefix = 'Right' if self.arm_name == 'rarm' else 'Left'
        rospy.loginfo('{} => '.format(prefix) + message)

    def logwarn(self, message):
        prefix = 'Right' if self.arm_name == 'rarm' else 'Left'
        rospy.logwarn('{} => '.format(prefix) + message)

    def post_init_hook(self, config: Config) -> None:
        robot_model = PR2()
        self.robot_model = robot_model
        self.robot_interface = self.robot_interface_type(robot_model)
        self.robot_model.angle_vector(self.robot_interface.angle_vector())
        self.gripper_close = False

        self.joy_manager.register_processor(
            JoyDataManager.Button.BOTTOM, self.move_gripper)

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
            tf_gripper2base_target.to_skrobot_coords(), end_effector, link_list)

        if isinstance(av_next, np.ndarray):
            self.robot_interface.angle_vector(av_next, time=0.8, time_scale=1.0)
        else:
            self.logwarn('solving inverse kinematics failed')

    def move_gripper(self) -> None:
        if self.gripper_close:
            self.robot_interface.move_gripper(self.arm_name, 0.06)  # type: ignore
            self.gripper_close = False
        else:
            self.robot_interface.move_gripper(self.arm_name, 0.00)  # type: ignore
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
        self.loginfo('calibrating controller for {}'.format(self.arm_name))
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

    def reset_to_home_position(self) -> None:
        self.is_tracking = False
        self.loginfo('turn off tracker')
        self.loginfo('resetting to home position')
        assert self.config.home_position is not None

        for joint_name in self.config.home_position.keys():
            angle = self.config.home_position[joint_name]
            self.robot_model.__dict__[joint_name].joint_angle(angle)
        self.robot_interface.angle_vector(self.robot_model.angle_vector(), time=3.0, time_scale=1.0)
        self.robot_interface.move_gripper('larm', self.config.home_position['l_gripper_joint'], effort=100)
        self.robot_interface.move_gripper('rarm', self.config.home_position['r_gripper_joint'], effort=100)
        self.robot_interface.wait_interpolation()


class PR2RightArmViveController(PR2ViveController):

    class RarmInterface(PR2ROSRobotInterface):
        def default_controller(self):
            return [self.rarm_controller]

    robot_interface_type = RarmInterface
    arm_joint_names: List[str] = rarm_joint_names
    arm_end_effector_name: str = 'r_gripper_tool_frame'
    arm_name: str = 'rarm'

    def __init__(self, config: Config, scale: float):
        super().__init__(config, scale, '/controller_LHR_FDF29FC7/joy', '/controller_LHR_FDF29FC7_as_posestamped')


class PR2LeftArmViveController(PR2ViveController):

    class LarmInterface(PR2ROSRobotInterface):
        def default_controller(self):
            return [self.larm_controller]

    robot_interface_type = LarmInterface
    arm_joint_names: List[str] = larm_joint_names
    arm_end_effector_name: str = 'l_gripper_tool_frame'
    arm_name: str = 'larm'

    def __init__(self, config: Config, scale: float):
        super().__init__(config, scale, '/controller_LHR_FF3DFFC7/joy', '/controller_LHR_FF3DFFC7_as_posestamped')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-pn', type=str, default=_default_project_name, help='project name')
    parser.add_argument('-scale', type=float, default=1.5, help='controller to real scaling')
    args = parser.parse_args()
    scale = args.scale
    config = Config.from_project_name(args.pn)

    rospy.init_node('pr2_vive_mohou')
    PR2RightArmViveController(config, scale)
    PR2LeftArmViveController(config, scale)
    rospy.spin()
