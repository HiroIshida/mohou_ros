import os
import numpy as np
import rosbag
from typing import Optional
from moviepy.editor import ImageSequenceClip
from mohou.types import RGBImage
from mohou_ros_utils.interpolator import NearestNeighbourInterpolator, AllSameInterpolationRule
from mohou_ros_utils.conversion import RGBImageConverter
from mohou_ros_utils.synclonizer import synclonize
from mohou_ros_utils.types import TimeStampedSequence
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.config import Config


def count_rosbag_file(config: Config) -> int:
    rosbag_dir = get_rosbag_dir(config.project_name)
    counter = 0
    for filename in os.listdir(rosbag_dir):
        _, ext = os.path.splitext(filename)
        if ext == '.bag':
            counter += 1
    return counter


def get_latest_rosbag_filename(config: Config) -> Optional[str]:
    rosbag_dir = get_rosbag_dir(config.project_name)
    rosbag_files = [f for f in os.listdir(rosbag_dir) if f.endswith(".bag")]
    filename_sorted = sorted(rosbag_files)
    if len(filename_sorted) == 0:
        return None
    return os.path.join(rosbag_dir, filename_sorted[-1])


def get_rosbag_filename(config: Config, postfix: str):
    rosbag_dir = get_rosbag_dir(config.project_name)
    filename = os.path.join(rosbag_dir, 'train-episode-{0}.bag'.format(postfix))
    return filename


def create_rosbag_command(filename: str, config: Config):
    cmd_rosbag = ['rosbag', 'record']
    topic_list = config.topics.rosbag_topic_list
    cmd_rosbag.extend(topic_list + ['/tf'])
    cmd_rosbag.extend(['--output-name', filename])
    print('subprocess cmd: {}'.format(cmd_rosbag))
    return cmd_rosbag


def bag2clip(bag: rosbag.Bag, config: Config, hz: float, speed: float) -> ImageSequenceClip:

    # def bgr2rgb(arr: np.ndarray) -> np.ndarray:
    #     return arr[..., ::-1].copy()

    converter = RGBImageConverter()
    seq = TimeStampedSequence.create_empty(RGBImage)
    rgb_config = config.topics.get_by_mohou_type(RGBImage)
    for topic, msg, t in bag.read_messages(topics=[rgb_config.name]):  # type: ignore
        seq.append(converter(msg), t.to_sec())
    rule = AllSameInterpolationRule(NearestNeighbourInterpolator)
    seq_regular = synclonize([seq], 1.0 / hz, itp_rule=rule)[0]
    # seq_numpy = [bgr2rgb(e.numpy()) for e in seq_regular.object_list]  # type: ignore
    seq_numpy = [e.numpy() for e in seq_regular.object_list]  # type: ignore
    fps = int(hz * speed)
    clip = ImageSequenceClip(seq_numpy, fps=fps)
    return clip
