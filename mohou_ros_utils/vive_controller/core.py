import time
from abc import ABC
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Generic, List, Optional, Type, TypeVar

import genpy
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

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


@dataclass
class ViveConfig:
    scale: float = 0.5
    timer_interval: float = 0.05
    joy_topic_name: str = ""
    pose_topic_name: str = ""

    def __post_init__(self):
        assert len(self.joy_topic_name) > 0
        assert len(self.pose_topic_name) > 0


ViveConfigT = TypeVar("ViveConfigT", bound="ViveConfig")


class ViveController(ABC, Generic[ViveConfigT]):
    config: ViveConfigT
    joy_manager: JoyDataManager
    pose_manager: PoseDataManager
    scale: float
    timer_interval: float
    is_initialized: bool
    is_tracking: bool

    def __init__(
        self, config: ViveConfigT, scale: float, joy_topic_name: str, pose_topic_name: str
    ):
        self.config = config
        self.joy_manager = JoyDataManager(config.joy_topic_name)
        self.pose_manager = PoseDataManager(config.pose_topic_name)
        self.scale = config.scale
        self.timer_interval = config.timer_interval

        rospy.Timer(rospy.Duration(self.timer_interval), self.on_timer)
        self.is_initialized = False
        self.is_tracking = False
        self.post_init_hook()

    def post_init_hook(self) -> None:
        pass

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
