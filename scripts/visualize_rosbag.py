#!/usr/bin/env python3
import argparse
import os

import rosbag

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.script_utils import bag2clip

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    parser.add_argument("-hz", type=float, default=5, help="sampling hz")
    parser.add_argument("-n", type=int, help="number of rosbags to be processed")
    parser.add_argument("-speed", type=float, default=3, help="x")
    parser.add_argument("-ext", type=str, default="mp4", help="gif or mp4")

    args = parser.parse_args()
    hz = args.hz
    n_gif_creation = args.n
    speed = args.speed
    ext_out = args.ext
    assert ext_out in ("mp4", "gif")
    config = Config.from_project_name(args.pn)

    rosbag_dir = get_rosbag_dir(config.project_name)

    filenames = os.listdir(rosbag_dir)
    if n_gif_creation is not None:
        filenames = filenames[:n_gif_creation]

    for filename in filenames:
        _, ext = os.path.splitext(filename)
        if ext != ".bag":
            continue

        full_path = os.path.join(rosbag_dir, filename)
        bag = rosbag.Bag(full_path)
        clip = bag2clip(bag, config, hz, speed)

        filename_raw, _ = os.path.splitext(filename)
        filename_out = "debug-" + filename_raw + "-hz{}-{}x".format(hz, speed) + "." + ext_out
        filename_out_full = os.path.join(rosbag_dir, filename_out)
        fps = int(hz * speed)
        if ext_out == "mp4":
            clip.write_videofile(filename_out_full, fps=clip.fps)
        else:
            clip.write_gif(filename_out_full, fps=clip.fps)
