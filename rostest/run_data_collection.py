#!/usr/bin/env python3
import subprocess
import unittest

import gdown
import rospy
from mohou.file import get_project_path

import rostest
from mohou_ros_utils.file import RelativeName, get_subpath


class TestNode(unittest.TestCase):
    project_name = "_mohou_ros_data_collection"

    def setUp(self):
        # create project_path
        project_path = get_project_path(self.project_name)
        project_path.mkdir(exist_ok=True)

        # download main_config
        def drive_url(file_id):
            url = "https://drive.google.com/uc?id={}".format(file_id)
            return url

        url = drive_url("1_d2ijjxXTzmsfADccuwY2t8DqaYC6OyK")
        main_config_path = get_subpath(project_path, RelativeName.main_config)
        gdown.download(url=url, output=str(main_config_path), quiet=True)

    @staticmethod
    def _run_command(cmd: str):
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)

    def test_pipeline(self):
        project_path = get_project_path(self.project_name)
        home_postion_path = get_subpath(project_path, RelativeName.home_position)

        assert project_path.exists()
        assert not home_postion_path.exists()
        cmd = "rosrun mohou_ros save_home_position.py -pn {}".format(self.project_name)
        self._run_command(cmd)
        assert home_postion_path.exists()


if __name__ == "__main__":
    rospy.init_node("test_data_collection")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
