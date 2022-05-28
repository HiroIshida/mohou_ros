#!/usr/bin/env python3
import argparse
import os
import rosbag
from moviepy.editor import ImageSequenceClip
from mohou.types import RGBImage

from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.conversion import RGBImageConverter
from mohou_ros_utils.config import Config
from mohou_ros_utils.types import TimeStampedSequence
from mohou_ros_utils.synclonizer import synclonize
from mohou_ros_utils.interpolator import NearestNeighbourInterpolator, AllSameInterpolationRule
from mohou_ros_utils import _default_project_name

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-pn', type=str, default=_default_project_name, help='project name')
    parser.add_argument('-hz', type=float, default=5, help='sampling hz')
    parser.add_argument('-speed', type=float, default=3, help='x')
    parser.add_argument('-ext', type=str, default='mp4', help='gif or mp4')

    args = parser.parse_args()
    hz = args.hz
    speed = args.speed
    ext_out = args.ext
    assert ext_out in ('mp4', 'gif')
    config = Config.from_project_name(args.pn)
    rgb_config = config.topics.get_by_mohou_type(RGBImage)

    rosbag_dir = get_rosbag_dir(config.project_name)
    for filename in os.listdir(rosbag_dir):
        _, ext = os.path.splitext(filename)
        if ext != '.bag':
            continue

        full_path = os.path.join(rosbag_dir, filename)
        bag = rosbag.Bag(full_path)
        converter = RGBImageConverter()
        seq = TimeStampedSequence.create_empty(RGBImage)
        for topic, msg, t in bag.read_messages(topics=[rgb_config.name]):  # type: ignore
            seq.append(converter(msg), t.to_sec())
        rule = AllSameInterpolationRule(NearestNeighbourInterpolator)
        seq_regular = synclonize([seq], 1.0 / hz, itp_rule=rule)[0]
        seq_numpy = [e.numpy() for e in seq_regular.object_list]  # type: ignore

        filename_raw, _ = os.path.splitext(filename)
        filename_out = 'debug-' + filename_raw + '-hz{}-{}x'.format(hz, speed) + '.' + ext_out
        filename_out_full = os.path.join(rosbag_dir, filename_out)
        fps = int(hz * speed)
        clip = ImageSequenceClip(seq_numpy, fps=fps)
        if ext_out == 'mp4':
            clip.write_videofile(filename_out_full, fps=fps)
        else:
            clip.write_gif(filename_out_full, fps=fps)
