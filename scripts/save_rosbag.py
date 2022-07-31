#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time
from typing import Optional

from mohou.file import get_project_path

from mohou_ros_utils.config import Config
from mohou_ros_utils.script_utils import create_rosbag_command, get_rosbag_filepath

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-t", type=float, default=120, help="time out")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn
    time_out: float = args.t

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    postfix = time.strftime("%Y%m%d%H%M%S")
    filepath = get_rosbag_filepath(project_path, postfix)
    cmd = create_rosbag_command(filepath, config)
    p = subprocess.Popen(cmd)

    def kill_process():
        os.kill(p.pid, signal.SIGTERM)
        time.sleep(1)  # a workaround
        print("killed process")

    ts = time.time()
    try:
        while True:
            time.sleep(0.5)
            elapsed = time.time() - ts
            if elapsed > time_out:
                kill_process()
                break
    except KeyboardInterrupt:
        kill_process()
