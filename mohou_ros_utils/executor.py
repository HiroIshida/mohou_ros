#!/usr/bin/env python3
from abc import ABC, abstractmethod
import os
from dataclasses import dataclass
import rospy
from typing import List, Optional
from sensor_msgs.msg import Image, JointState
import numpy as np
import matplotlib.pyplot as plt
import time
from moviepy.editor import ImageSequenceClip

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou_ros_utils.conversion import imgmsg_to_numpy
from mohou_ros_utils.file import create_if_not_exist
from mohou.propagator import Propagator
from mohou.default import create_default_propagator
from mohou.types import AngleVector, ElementDict, RGBImage
from mohou.utils import canvas_to_ndarray


@dataclass
class DebugImages:
    robot_camera: RGBImage
    network_input: RGBImage
    onestep_lookahaed_reconstructed: RGBImage

    def numpy(self) -> np.ndarray:
        fig = plt.figure()

        font = {'fontsize': 8, 'fontweight': 'medium'}
        ax1 = fig.add_subplot(1, 3, 1)
        ax1.imshow(self.robot_camera.numpy())
        ax1.set_title('robot camera', fontdict=font)

        ax2 = fig.add_subplot(1, 3, 2)
        ax2.imshow(self.network_input.numpy())
        ax2.set_title('network input', fontdict=font)

        ax3 = fig.add_subplot(1, 3, 3)
        ax3.imshow(self.onestep_lookahaed_reconstructed.numpy())
        ax3.set_title('reconstructed lookahead \n (one step)', fontdict=font)

        arr = canvas_to_ndarray(fig)
        plt.figure().clear(); plt.close(); plt.cla(); plt.clf()  # noqa
        return arr


class ExecutorBase(ABC):
    config: Config
    propagator: Propagator
    vconv: VersatileConverter
    control_joint_names: List[str]
    rgb_msg: Optional[Image] = None
    depth_msg: Optional[Image] = None
    joint_state_msg: Optional[JointState] = None
    current_av: Optional[AngleVector] = None
    running: bool = False
    dryrun: bool
    hz: float
    debug_images_seq: List[DebugImages]

    def __init__(self, config: Config, dryrun=True) -> None:
        n_joint = len(config.control_joints)
        propagator = create_default_propagator(config.project, n_joint)
        vconv = VersatileConverter.from_config(config)
        self.config = config
        self.propagator = propagator
        self.vconv = vconv
        self.control_joint_names = config.control_joints

        rospy.Subscriber(config.topics.rgb_topic_config.name, Image, self.on_rgb)
        rospy.Subscriber(config.topics.depth_topic_config.name, Image, self.on_depth)
        rospy.Subscriber(config.topics.av_topic_config.name, JointState, self.on_joint_state)

        self.post_init_hook()
        self.dryrun = dryrun

        # start!
        self.debug_images_seq = []
        self.hz = 1.0
        rospy.Timer(rospy.Duration(1.0 / self.hz), self.on_timer)
        self.running = True

    def on_rgb(self, msg: Image):
        self.rgb_msg = msg

    def on_depth(self, msg: Image):
        self.depth_msg = msg

    def on_joint_state(self, msg: JointState):
        self.joint_state_msg = msg

    def on_timer(self, event):
        if not self.running:
            return
        # TODO(HiroIshida) Currently consider only (RGBImage, AngleVector)

        if (self.joint_state_msg is None) or (self.rgb_msg is None):  # TODO(HiroIshida) depth!
            rospy.loginfo("cannot start because topics are not subscribed yet.")
            rospy.loginfo('joint state subscribed? : {}'.format(self.joint_state_msg is not None))
            rospy.loginfo('rgb msg subscribed? : {}'.format(self.rgb_msg is not None))
            return
        rospy.loginfo('on timer..')

        elems = [self.vconv(msg) for msg in [self.joint_state_msg, self.rgb_msg]]
        edict = ElementDict(elems)

        self.propagator.feed(edict)
        edict_next = self.propagator.predict(1)[0]

        robot_camera_view = RGBImage(imgmsg_to_numpy(self.rgb_msg))
        dimages = DebugImages(robot_camera_view, edict[RGBImage], edict_next[RGBImage])
        self.debug_images_seq.append(dimages)

        av_next_cand = edict_next[AngleVector]
        self.current_av = self.get_angle_vector()
        av_next = AngleVector((av_next_cand.numpy() - self.current_av.numpy()) * self.hz + self.current_av.numpy())  # type: ignore
        self.send_command(av_next)

    def on_termination(self):
        rospy.loginfo('Please hang tight. Creating a debug gif image...')
        self.running = False
        dir_name = os.path.join(self.config.get_project_dir(), 'execute_debug_gifs')
        create_if_not_exist(dir_name)
        file_name = os.path.join(dir_name, 'exec-{}.gif'.format(time.strftime("%Y%m%d%H%M%S")))
        clip = ImageSequenceClip([debug_images.numpy() for debug_images in self.debug_images_seq], fps=20)
        clip.write_gif(file_name, fps=20)

    @abstractmethod
    def post_init_hook(self) -> None:
        pass

    @abstractmethod
    def send_command(self, av: AngleVector) -> None:
        pass

    @abstractmethod
    def get_angle_vector(self) -> AngleVector:
        pass
