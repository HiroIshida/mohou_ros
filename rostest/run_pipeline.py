#!/usr/bin/env python3
import subprocess
import unittest

import gdown
import rospy
from mohou.file import get_project_path

import rostest


class TestNode(unittest.TestCase):
    project_name = "_mohou_ros_utils_test"

    def setUp(self):

        rospy.loginfo("prepare rosbag")

        def drive_url(file_id):
            url = "https://drive.google.com/uc?id={}".format(file_id)
            return url

        # prepare rosbag
        project_path = get_project_path(self.project_name)
        project_path.mkdir(exist_ok=True)

        url = drive_url("18EtBZHK1SIxgGMKOrITd_Eg5MK3KnQXA")
        zip_path = project_path / "rosbag.zip"
        gdown.download(url=url, output=str(zip_path), quiet=False)

        rosbag_path = project_path
        rosbag_path.mkdir(exist_ok=True)
        subprocess.call("unzip -o {} -d {}".format(zip_path, rosbag_path), shell=True)

        # prepare config files
        rospy.loginfo("prepare configs")
        url = drive_url("1LAF9JklX0NLUK_3isQOyFjNHSj-UzzWI")
        image_config_path = project_path / "image_config.yaml"
        gdown.download(url=url, output=str(image_config_path), quiet=True)

        url = drive_url("1_d2ijjxXTzmsfADccuwY2t8DqaYC6OyK")
        main_config_path = project_path / "main_config.yaml"
        gdown.download(url=url, output=str(main_config_path), quiet=True)

    def test_pipeline(self):

        # run commands
        cmd = "rosrun mohou_ros bags2chunk.py -hz 5 -remove_policy donothing -pn {}".format(
            self.project_name
        )
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)

        cmd = "rosrun mohou_ros bags2chunk.py -hz 20 -postfix autoencoder -remove_policy remove -pn {}".format(
            self.project_name
        )
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)

        cmd = "rosrun mohou_ros train {} 1 1".format(self.project_name)
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)


if __name__ == "__main__":
    rospy.init_node("test_sample")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
