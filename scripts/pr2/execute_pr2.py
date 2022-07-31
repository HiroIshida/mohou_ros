#!/usr/bin/env python3
import argparse
from typing import Optional

import rospy
from mohou.file import get_project_path

from mohou_ros_utils.pr2.executor import SkrobotPR2Executor

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("--force", action="store_true", help="disable dry option")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn
    force: bool = args.force

    rospy.init_node("executor", disable_signals=True)
    project_path = get_project_path(project_name)
    executor = SkrobotPR2Executor(project_path, dryrun=(not force))
    executor.run()

    try:
        while True:
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        rospy.loginfo("finish")
        executor.terminate()
