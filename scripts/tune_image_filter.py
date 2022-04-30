#!/usr/bin/env python3
import argparse
import os
import rosbag
import rospkg

from tunable_filter.composite_zoo import HSVBlurCropResolFilter
from mohou.types import RGBImage
from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.conversion import imgmsg_to_numpy


def get_first_rgb(config: Config):
    base_dir = get_rosbag_dir(config.project)
    for filename in os.listdir(base_dir):

        _, ext = os.path.splitext(filename)
        if ext != '.bag':
            continue

        rosbag_file = os.path.join(base_dir, filename)
        bag = rosbag.Bag(rosbag_file)
        for topic, msg, _ in bag.read_messages():
            if topic == config.topics.get_by_mohou_type(RGBImage).name:
                bag.close()
                return RGBImage(imgmsg_to_numpy(msg))
        bag.close()
    assert False


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))

    args = parser.parse_args()
    config_file = args.config
    config = Config.from_yaml_file(config_file)

    rgb = get_first_rgb(config)
    tunable = HSVBlurCropResolFilter.from_image(rgb.numpy())
    tunable.start_tuning(rgb.numpy())
    tunable.dump_yaml(config.get_image_config_path())
