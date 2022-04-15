#!/usr/bin/env python3
import argparse
import os
import rospy
import rospkg
from typing import Optional
from sensor_msgs.msg import Image, JointState

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou.propagator import Propagator, create_default_propagator
from mohou.types import AngleVector, ElementDict


class Executor:
    propagator: Propagator
    vconv: VersatileConverter
    rgb_msg: Optional[Image] = None
    depth_msg: Optional[Image] = None
    joint_state_msg: Optional[JointState] = None

    def __init__(self, config: Config) -> None:
        n_joint = len(config.control_joints)
        propagator = create_default_propagator(config.project, n_joint)
        vconv = VersatileConverter.from_config(config)
        self.propagator = propagator
        self.vconv = vconv

        rospy.Subscriber(config.topics.rgb_topic, Image, self.on_rgb)
        rospy.Subscriber(config.topics.depth_topic, Image, self.on_depth)
        rospy.Subscriber(config.topics.av_topic, JointState, self.on_joint_state)
        rospy.Timer(rospy.Duration(1.0 / config.hz), self.on_timer)

    def on_rgb(self, msg: Image):
        self.rgb_msg = msg

    def on_depth(self, msg: Image):
        self.depth_msg = msg

    def on_joint_state(self, msg: JointState):
        self.joint_state_msg = msg

    def on_timer(self, event):
        assert self.joint_state_msg is not None
        assert self.rgb_msg is not None
        assert self.depth_msg is not None

        elems = [self.vconv(msg) for msg in [self.joint_state_msg, self.rgb_msg, self.depth_msg]]
        edict = ElementDict(elems)
        self.propagator.feed(edict)
        edict_next = self.propagator.predict(1)[0]
        av_next = edict_next[AngleVector]
        print(av_next)


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))

    args = parser.parse_args()
    config = Config.from_yaml_file(args.config)

    rospy.init_node('executor')
    executor = Executor(config)
