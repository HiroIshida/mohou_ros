#!/usr/bin/env python3
import shutil
import subprocess
import unittest
from pathlib import Path

import rospy
from mohou.file import get_project_path

import rostest
from mohou_ros_utils.file import RelativeName, get_subpath


class TestNode(unittest.TestCase):
    project_name = "_mohou_ros_pipeline"

    @property
    def project_path(self) -> Path:
        project_path = get_project_path(self.project_name)
        return project_path

    def setUp(self):
        pass

    @staticmethod
    def _run_command(cmd: str):
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)

    def _test_executor(self):

        # clean up execution result directory
        exec_debug_dir_path = get_subpath(self.project_path, RelativeName.exec_debug)
        shutil.rmtree(exec_debug_dir_path)
        exec_debug_dir_path.mkdir(exist_ok=True)

        # test start
        cmd = "rosrun mohou_ros reset_to_home.py -pn {}".format(self.project_name)
        self._run_command(cmd)

        cmd = "rosrun mohou_ros execute_pr2.py -pn {} --force -t 10".format(self.project_name)
        self._run_command(cmd)

        file_path_list = list(exec_debug_dir_path.iterdir())

        rosbag_path_list = [p for p in file_path_list if p.name.endswith(".bag")]
        assert len(rosbag_path_list) == 1

        mp4_path_list = [p for p in file_path_list if p.name.endswith(".mp4")]
        assert len(mp4_path_list) == 2

        pkl_path_list = [p for p in file_path_list if p.name.endswith(".pkl")]
        assert len(pkl_path_list) == 1

    def test_pipeline(self):
        self._test_executor()


if __name__ == "__main__":
    rospy.init_node("test_executor")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
