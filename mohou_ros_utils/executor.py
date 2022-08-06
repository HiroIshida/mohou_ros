#!/usr/bin/env python3
import os
import pickle
import signal
import subprocess
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
import rosbag
import rospy
import torch
from mohou.default import auto_detect_autoencoder_type, create_default_propagator
from mohou.model.autoencoder import AutoEncoderBase
from mohou.propagator import Propagator
from mohou.trainer import TrainCache
from mohou.types import AngleVector, ElementDict, GripperState, RGBImage, TerminateFlag
from mohou.utils import canvas_to_ndarray
from moviepy.editor import ImageSequenceClip
from pr2_controllers_msgs.msg import JointControllerState
from sensor_msgs.msg import CompressedImage, Image, JointState

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import MessageConverterCollection
from mohou_ros_utils.file import RelativeName, get_subpath
from mohou_ros_utils.script_utils import bag2clip, create_rosbag_command


@dataclass
class DebugImages:
    robot_camera: RGBImage
    network_input: RGBImage
    reconstructed: RGBImage
    onestep_lookahaed_reconstructed: RGBImage

    def numpy(self) -> np.ndarray:
        fig = plt.figure()

        def bgr2rgb(arr: np.ndarray) -> np.ndarray:
            return arr[..., ::-1].copy()

        font = {"fontsize": 8, "fontweight": "medium"}
        ax1 = fig.add_subplot(1, 4, 1)
        ax1.imshow(bgr2rgb(self.robot_camera.numpy()))
        ax1.set_title("robot camera", fontdict=font)

        ax2 = fig.add_subplot(1, 4, 2)
        ax2.imshow(bgr2rgb(self.network_input.numpy()))
        ax2.set_title("network input", fontdict=font)

        ax3 = fig.add_subplot(1, 4, 3)
        ax3.imshow(bgr2rgb(self.reconstructed.numpy()))
        ax3.set_title("network output reconstructed", fontdict=font)

        ax3 = fig.add_subplot(1, 4, 4)
        ax3.imshow(bgr2rgb(self.onestep_lookahaed_reconstructed.numpy()))
        ax3.set_title("reconstructed lookahead \n (one step)", fontdict=font)

        arr = canvas_to_ndarray(fig)
        plt.figure().clear()
        plt.close()
        plt.cla()
        plt.clf()  # noqa
        return arr


class ExecutorBase(ABC):
    config: Config
    propagator: Propagator
    autoencoder: AutoEncoderBase
    conv: MessageConverterCollection
    control_joint_names: List[str]
    running: bool
    dryrun: bool
    hz: float
    debug_images_seq: List[DebugImages]
    edict_seq: List[ElementDict]
    is_terminatable: bool
    str_time_postfix: str

    rgb_topic_name: str
    gripper_topic_name: str
    av_topic_name: str

    rosbag_cmd_popen: Optional[subprocess.Popen] = None
    rgb_msg: Optional[CompressedImage] = None
    joint_state_msg: Optional[JointState] = None
    joint_cont_state_msg: Optional[JointControllerState] = None
    current_av: Optional[AngleVector] = None

    def __init__(self, project_path: Path, dryrun=True, save_rosbag=True) -> None:
        propagator: Propagator = create_default_propagator(project_path, Propagator)

        ae_type = auto_detect_autoencoder_type(project_path)
        tcache_autoencoder = TrainCache.load(project_path, ae_type)
        assert tcache_autoencoder.best_model is not None
        self.autoencoder = tcache_autoencoder.best_model

        config = Config.from_project_path(project_path)
        conv = MessageConverterCollection.from_config(config)

        self.config = config
        self.propagator = propagator
        self.conv = conv
        self.control_joint_names = config.control_joints
        self.rgb_topic_name = config.topics.get_by_mohou_type(RGBImage).name
        self.av_topic_name = config.topics.get_by_mohou_type(AngleVector).name
        self.gripper_topic_name = config.topics.get_by_mohou_type(GripperState).name

        rospy.Subscriber(self.av_topic_name, JointState, self.on_joint_state)
        rospy.Subscriber(self.rgb_topic_name, CompressedImage, self.on_rgb)
        rospy.Subscriber(self.gripper_topic_name, JointControllerState, self.on_joint_cont_state)

        self._post_init()
        self.dryrun = dryrun

        # start!
        self.debug_images_seq = []
        self.edict_seq = []
        self.hz = 1.0
        rospy.Timer(rospy.Duration(1.0 / self.hz), self.on_timer)
        self.running = False
        self.is_terminatable = False
        self.str_time_postfix = time.strftime("%Y%m%d%H%M%S")

        if save_rosbag:

            debug_data_path = get_subpath(project_path, RelativeName.exec_debug)
            rosbag_file_path = debug_data_path / "backup-{}.bag".format(self.str_time_postfix)
            cmd = create_rosbag_command(rosbag_file_path, config)
            self.rosbag_cmd_popen = subprocess.Popen(cmd)
        else:
            self.rosbag_cmd_popen = None

    def run(self):
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

        if (
            (self.joint_state_msg is None)
            or (self.joint_cont_state_msg is None)
            or (self.rgb_msg is None)
        ):
            rospy.loginfo("cannot start because topics are not subscribed yet.")
            rospy.loginfo("joint state subscribed? : {}".format(self.joint_state_msg is not None))
            rospy.loginfo(
                "joint cont state subscribed? : {}".format(self.joint_cont_state_msg is not None)
            )
            rospy.loginfo("rgb msg subscribed? : {}".format(self.rgb_msg is not None))
            return
        rospy.loginfo("on timer..")

        msg_table = {
            self.av_topic_name: self.joint_state_msg,
            self.rgb_topic_name: self.rgb_msg,
            self.gripper_topic_name: self.joint_cont_state_msg,
        }

        edict_current = self.conv.apply_to_msg_table(msg_table)

        self.propagator.feed(edict_current)

        edict_next = self.propagator.predict(1)[0]
        self.is_terminatable = edict_next[TerminateFlag].numpy().item() > 0.98

        # save debug infos
        robot_camera_view = edict_current[RGBImage]

        inp = torch.unsqueeze(edict_current[RGBImage].to_tensor(), dim=0)
        reconstructed = RGBImage.from_tensor(self.autoencoder.forward(inp).squeeze(dim=0))

        dimages = DebugImages(
            robot_camera_view,
            edict_current[RGBImage],
            reconstructed,
            edict_next[RGBImage],
        )
        self.debug_images_seq.append(dimages)
        self.edict_seq.append(edict_current)

        self.send_command(edict_next, edict_current)

    def terminate(self, dump_debug_info: bool = True):
        self.running = False

        debug_data_path = get_subpath(self.config.project_path, RelativeName.exec_debug)

        if dump_debug_info:
            rospy.loginfo("Please hang tight. Creating a debug debug video...")
            video_file_path = debug_data_path / "images-{}.mp4".format(self.str_time_postfix)
            debug_image_clip = ImageSequenceClip(
                [debug_images.numpy() for debug_images in self.debug_images_seq], fps=20
            )
            debug_image_clip.write_videofile(str(video_file_path), fps=20)

            rospy.loginfo("Please hang tight. Saving debug edict sequence")
            edict_file_path = debug_data_path / "edicts-{}.pkl".format(self.str_time_postfix)
            with edict_file_path.open(mode="wb") as f:
                pickle.dump(self.edict_seq, f)

            is_rosbag_recorded = self.rosbag_cmd_popen is not None
            if is_rosbag_recorded:
                assert self.rosbag_cmd_popen is not None  # just for mypy
                os.kill(self.rosbag_cmd_popen.pid, signal.SIGTERM)
                self.rosbag_cmd_popen.wait(timeout=10)
                time.sleep(2)  # maybe takes a little bit to save rosbag

                # get rosbag_filename
                assert self.rosbag_cmd_popen is not None
                rosbag_filename = None
                for i, arg in enumerate(self.rosbag_cmd_popen.args):
                    if arg == "-O" or arg == "--output-name":
                        rosbag_filename = self.rosbag_cmd_popen.args[i + 1]
                assert rosbag_filename is not None
                bag = rosbag.Bag(rosbag_filename)
                movie_clip = bag2clip(bag, self.config, hz=10, speed=1.0)
                bag_video_file_path = debug_data_path / "video-{}.mp4".format(self.str_time_postfix)
                movie_clip.write_videofile(str(bag_video_file_path))

    @abstractmethod
    def _post_init(self) -> None:
        pass

    @abstractmethod
    def send_command(self, edict_next: ElementDict, edict_current: ElementDict) -> None:
        pass

    @abstractmethod
    def get_angle_vector(self) -> AngleVector:
        pass


@dataclass
class SequentialExecutor:
    executors: List[ExecutorBase]
    executor_current: Optional[ExecutorBase]
    debug_images_seq: List[DebugImages]
    edict_seq: List[ElementDict]

    @classmethod
    def from_executors(cls, executors: List[ExecutorBase]) -> "SequentialExecutor":
        return cls(executors, None, [], [])

    def __post_init__(self):
        for e in self.executors:
            assert not e.dryrun

    def run(self) -> None:
        for idx, executor in enumerate(self.executors):
            rospy.loginfo("start executor {}".format(idx))
            self.executor_current = executor
            executor.run()
            while True:
                time.sleep(0.5)
                if executor.is_terminatable:
                    break
            executor.terminate(dump_debug_info=False)
            self.debug_images_seq.extend(executor.debug_images_seq)
            self.edict_seq.extend(executor.edict_seq)
            rospy.loginfo("stop executor {}".format(idx))
        rospy.loginfo("executed all execturos")

    def terminate(self) -> None:
        assert self.executor_current is not None
        self.executor_current.terminate(dump_debug_info=False)
