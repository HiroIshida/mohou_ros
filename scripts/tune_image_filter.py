#!/usr/bin/env python3
import argparse
import os
import yaml
from dataclasses import dataclass
from copy import deepcopy
import cv2
import numpy as np
import rosbag
import rospkg

from mohou.types import RGBImage
from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.file import get_image_config_file
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
            if topic == config.topics.rgb_topic_config.name:
                bag.close()
                return RGBImage(imgmsg_to_numpy(msg))
        bag.close()
    assert False


@dataclass
class CropFilter:
    x_max: int
    y_max: int
    window_name = 'window'
    x_min_tune: int = -1
    x_max_tune: int = -1
    y_min_tune: int = -1
    y_max_tune: int = -1

    def __post_init__(self):
        cv2.createTrackbar('x_min', self.window_name, 0, self.x_max, lambda x: None)
        cv2.createTrackbar('x_max', self.window_name, 0, self.x_max, lambda x: None)
        cv2.createTrackbar('y_min', self.window_name, 0, self.y_max, lambda x: None)
        cv2.createTrackbar('y_max', self.window_name, 0, self.y_max, lambda x: None)

    def __call__(self, rgb: RGBImage) -> np.ndarray:
        arr = np.ones(rgb.shape[:2], dtype=bool)
        arr[:self.x_min_tune, :] = False
        arr[self.x_max_tune:, :] = False
        arr[:, :self.y_min_tune] = False
        arr[:, self.y_max_tune:] = False
        return arr

    def reflect_trackbar(self):
        self.x_min_tune = cv2.getTrackbarPos('x_min', self.window_name)
        self.x_max_tune = cv2.getTrackbarPos('x_max', self.window_name)
        self.y_min_tune = cv2.getTrackbarPos('y_min', self.window_name)
        self.y_max_tune = cv2.getTrackbarPos('y_max', self.window_name)

    def dump_yaml(self, config: Config):
        config_file = get_image_config_file(config.project)
        dic = {'x_min': self.x_min_tune, 'x_max': self.x_max_tune, 'y_min': self.y_min_tune, 'y_max': self.y_max_tune}
        with open(config_file, 'w') as f:
            yaml.dump(dic, f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))

    args = parser.parse_args()
    config_file = args.config
    config = Config.from_yaml_file(config_file)

    rgb = get_first_rgb(config)

    window_name = 'window'
    cv2.namedWindow(window_name)
    width, height, _ = rgb.shape
    F = CropFilter(width, height)

    while True:
        seg = F(rgb)
        img_show = deepcopy(rgb.numpy())
        img_show[np.logical_not(seg)] = (0, 0, 0)
        cv2.imshow('window', img_show)
        F.reflect_trackbar()

        if cv2.waitKey(50) == ord('q'):
            break

    F.dump_yaml(config)
