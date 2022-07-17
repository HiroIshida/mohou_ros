#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.script_utils import create_rosbag_command, get_rosbag_filename

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")

    args = parser.parse_args()
    config = Config.from_project_name(args.pn)

    postfix = time.strftime("%Y%m%d%H%M%S")
    filename = get_rosbag_filename(config, postfix)
    cmd = create_rosbag_command(filename, config)
    p = subprocess.Popen(cmd)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        os.kill(p.pid, signal.SIGKILL)
        time.sleep(1)  # a workaround
