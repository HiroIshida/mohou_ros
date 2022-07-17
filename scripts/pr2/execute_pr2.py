#!/usr/bin/env python3
import argparse

import rospy

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.pr2.executor import SkrobotPR2Executor

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    parser.add_argument("--force", action="store_true", help="disable dry option")

    args = parser.parse_args()
    project_name = args.pn
    force = args.force

    rospy.init_node("executor", disable_signals=True)
    executor = SkrobotPR2Executor(project_name, dryrun=(not force))
    executor.run()

    try:
        while True:
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        rospy.loginfo("finish")
        executor.terminate()
