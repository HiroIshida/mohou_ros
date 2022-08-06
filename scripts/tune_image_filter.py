#!/usr/bin/env python3
import argparse
import os
from typing import Optional

import rosbag
from mohou.file import get_project_path
from mohou.types import RGBImage
from tunable_filter.composite_zoo import HSVBlurCropResolFilter
from tunable_filter.tunable import ResolutionChangeResizer

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import RGBImageConverter
from mohou_ros_utils.file import RelativeName, get_subpath


def get_first_rgb(config: Config) -> RGBImage:
    rgb_conv = RGBImageConverter.from_config(config)
    rosbag_path = get_subpath(project_path, RelativeName.rosbag)

    for filepath in rosbag_path.iterdir():

        _, ext = os.path.splitext(filepath)
        if ext != ".bag":
            continue

        bag = rosbag.Bag(str(filepath))
        for topic, msg, _ in bag.read_messages():
            if topic == config.topics.get_by_mohou_type(RGBImage).name:
                bag.close()
                return rgb_conv.apply((msg,))
        bag.close()
    assert False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument(
        "--testrun", action="store_true", help="not using gui. just save the default value"
    )
    args = parser.parse_args()

    project_name: Optional[str] = args.pn
    is_testrun = args.testrun

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    rgb = get_first_rgb(config)
    tunable = HSVBlurCropResolFilter.from_image(rgb.numpy())
    tunable.set_value(ResolutionChangeResizer, "resol", 112)

    image_config_path = get_subpath(project_path, RelativeName.image_config)

    if is_testrun:
        tunable.dump_yaml(str(image_config_path))
    else:
        tunable.launch_window()
        tunable.start_tuning(
            rgb.numpy(),
            callback=lambda this: this.dump_yaml(str(image_config_path)),
        )
