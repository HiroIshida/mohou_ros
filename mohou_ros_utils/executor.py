#!/usr/bin/env python3
from abc import ABC, abstractmethod
import os
import pickle
from dataclasses import dataclass
import rospy
from typing import List, Optional
from sensor_msgs.msg import CompressedImage, Image, JointState
import numpy as np
import matplotlib.pyplot as plt
import time
from moviepy.editor import ImageSequenceClip
from pr2_controllers_msgs.msg import JointControllerState

from mohou.propagator import Propagator
from mohou.default import create_default_propagator
from mohou.types import AngleVector, ElementDict, RGBImage, GripperState
from mohou.utils import canvas_to_ndarray
from mohou.file import get_project_dir

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou_ros_utils.file import create_if_not_exist


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
    project_name: str
    config: Config
    propagator: Propagator
    vconv: VersatileConverter
    control_joint_names: List[str]
    rgb_msg: Optional[CompressedImage] = None
    joint_state_msg: Optional[JointState] = None
    joint_cont_state_msg: Optional[JointControllerState] = None
    current_av: Optional[AngleVector] = None
    running: bool = False
    dryrun: bool
    hz: float
    debug_images_seq: List[DebugImages]
    debug_edict_seq: List[ElementDict]

    def __init__(self, project_name: str, dryrun=True) -> None:
        propagator = create_default_propagator(project_name)
        config = Config.from_project_name(project_name)
        vconv = VersatileConverter.from_config(config)

        self.project_name = project_name
        self.config = config
        self.propagator = propagator
        self.vconv = vconv
        self.control_joint_names = config.control_joints

        rospy.Subscriber(config.topics.get_by_mohou_type(RGBImage).name, CompressedImage, self.on_rgb)
        rospy.Subscriber(config.topics.get_by_mohou_type(GripperState).name, JointControllerState, self.on_joint_cont_state)

        self.post_init_hook()
        self.dryrun = dryrun

        # start!
        self.debug_images_seq = []
        self.debug_edict_seq = []
        self.hz = 1.0
        rospy.Timer(rospy.Duration(1.0 / self.hz), self.on_timer)
        self.running = True

    def on_rgb(self, msg: CompressedImage):
        self.rgb_msg = msg

    def on_depth(self, msg: Image):
        self.depth_msg = msg

    def on_joint_state(self, msg: JointState):
        self.joint_state_msg = msg

    def on_joint_cont_state(self, msg: JointControllerState):
        self.joint_cont_state_msg = msg

    def on_timer(self, event):
        if not self.running:
            return

        if (self.joint_state_msg is None) or (self.joint_cont_state_msg is None) or (self.rgb_msg is None):
            rospy.loginfo("cannot start because topics are not subscribed yet.")
            rospy.loginfo('joint state subscribed? : {}'.format(self.joint_state_msg is not None))
            rospy.loginfo('joint cont state subscribed? : {}'.format(self.joint_cont_state_msg is not None))
            rospy.loginfo('rgb msg subscribed? : {}'.format(self.rgb_msg is not None))
            return
        rospy.loginfo('on timer..')

        elems = [self.vconv(msg) for msg in [self.joint_state_msg, self.rgb_msg, self.joint_cont_state_msg]]
        edict_current = ElementDict(elems)

        self.propagator.feed(edict_current)

        edict_next = self.propagator.predict(1)[0]

        # save debug infos
        robot_camera_view = edict_current[RGBImage]
        dimages = DebugImages(robot_camera_view, edict_current[RGBImage], edict_next[RGBImage])
        self.debug_images_seq.append(dimages)
        self.debug_edict_seq.append(edict_current)

        self.send_command(edict_next, edict_current)

    def on_termination(self):
        self.running = False
        dir_name = os.path.join(get_project_dir(self.project_name), 'execution_debug_data')
        str_time = time.strftime("%Y%m%d%H%M%S")
        create_if_not_exist(dir_name)

        rospy.loginfo('Please hang tight. Creating a debug gif image...')
        file_name = os.path.join(dir_name, 'images-{}.gif'.format(str_time))
        clip = ImageSequenceClip([debug_images.numpy() for debug_images in self.debug_images_seq], fps=20)
        clip.write_gif(file_name, fps=20)

        rospy.loginfo('Please hang tight. Saving debug edict sequence')
        file_name = os.path.join(dir_name, 'edicts-{}.pkl'.format(str_time))
        with open(file_name, 'wb') as f:
            pickle.dump(self.debug_edict_seq, f)

    @abstractmethod
    def post_init_hook(self) -> None:
        pass

    @abstractmethod
    def send_command(self, edict_next: ElementDict, edict_current: ElementDict) -> None:
        pass

    @abstractmethod
    def get_angle_vector(self) -> AngleVector:
        pass
