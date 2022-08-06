#!/usr/bin/env python3
import argparse
import time
from typing import Optional

import rospy
from mohou.file import get_project_path

from mohou_ros_utils.pr2.executor import (
    EusPR2Executor,
    ExecutorBase,
    SkrobotPR2Executor,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-t", type=float, default=300, help="time out")
    parser.add_argument("--force", action="store_true", help="disable dry option")
    parser.add_argument("--roseus", action="store_true", help="command via roseus")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn
    time_out: float = args.t
    force: bool = args.force
    roseus: bool = args.roseus

    rospy.init_node("executor", disable_signals=True)
    project_path = get_project_path(project_name)

    if roseus:
        executor: ExecutorBase = EusPR2Executor(project_path, dryrun=(not force))
    else:
        executor = SkrobotPR2Executor(project_path, dryrun=(not force))

    executor.run()

    ts = time.time()
    try:
        while True:
            rospy.rostime.wallsleep(0.5)
            elapsed = time.time() - ts
            if elapsed > time_out:
                rospy.loginfo("finish")
                executor.terminate()
                break
    except KeyboardInterrupt:
        rospy.loginfo("finish")
        executor.terminate()
