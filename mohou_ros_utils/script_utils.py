import os
from pathlib import Path
from typing import List, Optional

import numpy as np
import rosbag
from mohou.types import RGBImage
from moviepy.editor import ImageSequenceClip

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import RGBImageConverter
from mohou_ros_utils.file import RelativeName, get_subpath
from mohou_ros_utils.interpolator import (
    AllSameInterpolationRule,
    NearestNeighbourInterpolator,
)
from mohou_ros_utils.synclonizer import synclonize
from mohou_ros_utils.types import TimeStampedSequence


def get_rosbag_paths(project_path: Path) -> List[Path]:
    rosbag_dir_path = get_subpath(project_path, RelativeName.rosbag)
    paths = []
    for filepath in rosbag_dir_path.iterdir():
        _, ext = os.path.splitext(filepath)
        if ext == ".bag":
            paths.append(filepath)
    return paths


def count_rosbag_file(project_path: Path) -> int:
    paths = get_rosbag_paths(project_path)
    return len(paths)


def get_latest_rosbag_filename(project_path: Path) -> Optional[Path]:
    """
    NOTE: this function assumes that rosbag file has time-stamp postfix
    """
    paths = get_rosbag_paths(project_path)
    if len(paths) == 0:
        return None
    names_sorted = sorted([str(p) for p in paths])
    return Path(names_sorted[-1])


def get_rosbag_filepath(project_path: Path, postfix: str) -> Path:
    rosbag_dir_path = get_subpath(project_path, RelativeName.rosbag)
    filepath = rosbag_dir_path / "train-episode-{0}.bag".format(postfix)
    return filepath


def create_rosbag_command(project_path: Path, config: Config):
    cmd_rosbag = ["rosbag", "record"]
    topic_list = config.topics.rosbag_topic_list
    cmd_rosbag.extend(topic_list + ["/tf"])
    cmd_rosbag.extend(["--output-name", str(project_path)])
    print("subprocess cmd: {}".format(cmd_rosbag))
    return cmd_rosbag


def bag2clip(bag: rosbag.Bag, config: Config, hz: float, speed: float) -> ImageSequenceClip:
    def bgr2rgb(arr: np.ndarray) -> np.ndarray:
        return arr[..., ::-1].copy()

    converter = RGBImageConverter.from_config_topic_name_only(config)
    seq = TimeStampedSequence.create_empty(RGBImage)
    rgb_config = config.topics.get_by_mohou_type(RGBImage)
    for topic, msg, t in bag.read_messages(topics=[rgb_config.name]):  # type: ignore
        seq.append(converter.apply(msg), t.to_sec())
    rule = AllSameInterpolationRule(NearestNeighbourInterpolator)
    seq_regular = synclonize([seq], 1.0 / hz, itp_rule=rule)[0]
    seq_numpy = [bgr2rgb(e.numpy()) for e in seq_regular.object_list]  # type: ignore
    fps = int(hz * speed)
    clip = ImageSequenceClip(seq_numpy, fps=fps)
    return clip
