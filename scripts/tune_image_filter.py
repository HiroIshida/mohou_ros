#!/usr/bin/env python3
import argparse
import os

import rosbag
from mohou.types import RGBImage
from tunable_filter.composite_zoo import HSVBlurCropResolFilter
from tunable_filter.tunable import ResolutionChangeResizer

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import RGBImageConverter
from mohou_ros_utils.file import get_image_config_path, get_rosbag_dir


def get_first_rgb(config: Config) -> RGBImage:
    rgb_conv = RGBImageConverter()
    base_dir = get_rosbag_dir(config.project_name)
    for filename in os.listdir(base_dir):

        _, ext = os.path.splitext(filename)
        if ext != ".bag":
            continue

        rosbag_file = os.path.join(base_dir, filename)
        bag = rosbag.Bag(rosbag_file)
        for topic, msg, _ in bag.read_messages():
            if topic == config.topics.get_by_mohou_type(RGBImage).name:
                bag.close()
                return rgb_conv(msg)
        bag.close()
    assert False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    args = parser.parse_args()
    project_name = args.pn
    config = Config.from_project_name(project_name)

    rgb = get_first_rgb(config)
    tunable = HSVBlurCropResolFilter.from_image(rgb.numpy())
    tunable.set_value(ResolutionChangeResizer, "resol", 112)

    tunable.start_tuning(
        rgb.numpy(),
        callback=lambda this: this.dump_yaml(get_image_config_path(project_name)),
    )
