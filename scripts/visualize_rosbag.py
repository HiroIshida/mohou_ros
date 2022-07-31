#!/usr/bin/env python3
import argparse
import os
from typing import Optional

import rosbag
from mohou.file import get_project_path

from mohou_ros_utils.config import Config
from mohou_ros_utils.file import RelativeName, get_subpath
from mohou_ros_utils.script_utils import bag2clip, get_rosbag_paths

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-hz", type=float, default=5, help="sampling hz")
    parser.add_argument("-n", type=int, help="number of rosbags to be processed")
    parser.add_argument("-speed", type=float, default=3, help="x")
    parser.add_argument("-ext", type=str, default="mp4", help="gif or mp4")

    args = parser.parse_args()
    hz: float = args.hz
    n_gif_creation: Optional[int] = args.n
    speed: float = args.speed
    ext_out: str = args.ext
    project_name: Optional[str] = args.pn

    assert ext_out in ("mp4", "gif")

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    rosbag_dir_path = get_subpath(project_path, RelativeName.rosbag)
    rosbag_paths = get_rosbag_paths(project_path)
    if n_gif_creation is not None:
        rosbag_paths = rosbag_paths[:n_gif_creation]

    for rosbag_path in get_rosbag_paths(project_path):
        bag = rosbag.Bag(str(rosbag_path))
        clip = bag2clip(bag, config, hz, speed)

        filename_raw, _ = os.path.splitext(rosbag_path.name)
        filename_out = "debug-" + filename_raw + "-hz{}-{}x".format(hz, speed) + "." + ext_out

        file_path = rosbag_dir_path / filename_out
        fps = int(hz * speed)
        if ext_out == "mp4":
            clip.write_videofile(str(file_path), fps=clip.fps)
        else:
            clip.write_gif(str(file_path), fps=clip.fps)
