#!/usr/bin/env python3
from abc import ABC, abstractmethod
import rospy
from typing import List, Optional
from sensor_msgs.msg import Image, JointState

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou_ros_utils.conversion import numpy_to_imgmsg
from mohou.propagator import Propagator, create_default_propagator
from mohou.types import AngleVector, ElementDict, RGBImage


class ExecutorBase(ABC):
    propagator: Propagator
    vconv: VersatileConverter
    control_joint_names: List[str]
    rgb_msg: Optional[Image] = None
    depth_msg: Optional[Image] = None
    joint_state_msg: Optional[JointState] = None
    current_av: Optional[AngleVector] = None
    dryrun: bool

    def __init__(self, config: Config, dryrun=True) -> None:
        n_joint = len(config.control_joints)
        propagator = create_default_propagator(config.project, n_joint)
        vconv = VersatileConverter.from_config(config)
        self.propagator = propagator
        self.vconv = vconv
        self.control_joint_names = config.control_joints

        rospy.Subscriber(config.topics.rgb_topic_config.name, Image, self.on_rgb)
        rospy.Subscriber(config.topics.depth_topic_config.name, Image, self.on_depth)
        rospy.Subscriber(config.topics.av_topic_config.name, JointState, self.on_joint_state)

        self.pub_ae_inp = rospy.Publisher('network_image_inp', Image, 10)
        self.pub_ae_out = rospy.Publisher('network_image_out', Image, 10)

        self.post_init_hook()
        self.dryrun = dryrun

        # start!
        rospy.Timer(rospy.Duration(0.5), self.on_timer)

    def on_rgb(self, msg: Image):
        self.rgb_msg = msg

    def on_depth(self, msg: Image):
        self.depth_msg = msg

    def on_joint_state(self, msg: JointState):
        self.joint_state_msg = msg

    def on_timer(self, event):
        if (self.joint_state_msg is None) or (self.rgb_msg is None):  # TODO(HiroIshida) depth!
            rospy.loginfo("cannot start because topics are not subscribed yet.")
            rospy.loginfo('joint state subscribed? : {}'.format(self.joint_state_msg is not None))
            rospy.loginfo('rgb msg subscribed? : {}'.format(self.rgb_msg is not None))
            return
        rospy.loginfo('on timer..')

        elems = [self.vconv(msg) for msg in [self.joint_state_msg, self.rgb_msg]]
        edict = ElementDict(elems)

        self.pub_ae_inp.publish(numpy_to_imgmsg(edict[RGBImage].numpy(), self.rgb_msg.encoding))

        self.propagator.feed(edict)
        edict_next = self.propagator.predict(1)[0]

        self.pub_ae_out.publish(numpy_to_imgmsg(edict_next[RGBImage].numpy(), self.rgb_msg.encoding))

        av_next_cand = edict_next[AngleVector]
        self.current_av = self.get_angle_vector()
        av_next = AngleVector((av_next_cand.numpy() - self.current_av.numpy()) * 0.5 + self.current_av.numpy())  # type: ignore
        self.send_command(av_next)

    @abstractmethod
    def post_init_hook(self) -> None:
        pass

    @abstractmethod
    def send_command(self, av: AngleVector) -> None:
        pass

    @abstractmethod
    def get_angle_vector(self) -> AngleVector:
        pass
