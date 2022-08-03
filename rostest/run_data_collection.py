#!/usr/bin/env python3
import shutil
import subprocess
import time
import unittest
from pathlib import Path

import rospkg
import rospy
from mohou.file import get_project_path
from mohou.utils import assert_equal_with_message
from rosbag import Bag

import rostest
from mohou_ros_utils.config import Config
from mohou_ros_utils.file import RelativeName, get_subpath
from mohou_ros_utils.rosbag import bag_to_seqs
from mohou_ros_utils.script_utils import get_rosbag_paths


class TestNode(unittest.TestCase):
    project_name = "_mohou_ros_pipeline"

    @property
    def project_path(self) -> Path:
        project_path = get_project_path(self.project_name)
        return project_path

    def setUp(self):
        time.sleep(3)  # wait a bit for other launch files
        # create project_path
        if self.project_path.exists():
            shutil.rmtree(self.project_path)
        self.project_path.mkdir(exist_ok=True)

        pack_path = Path(rospkg.RosPack().get_path("mohou_ros"))
        config_orig_path = pack_path / "rostest/config/main_config.yaml"

        main_config_path = get_subpath(self.project_path, RelativeName.main_config)
        if main_config_path.is_symlink():
            main_config_path.unlink()
        main_config_path.symlink_to(config_orig_path)

    @staticmethod
    def _run_command(cmd: str):
        rospy.loginfo(cmd)
        subprocess.run(cmd, shell=True)

    def _test_save_home_position(self):
        home_postion_path = get_subpath(self.project_path, RelativeName.home_position)

        assert self.project_path.exists()
        assert not home_postion_path.exists()
        cmd = "rosrun mohou_ros save_home_position.py -pn {}".format(self.project_name)
        self._run_command(cmd)
        assert home_postion_path.exists()

    def _test_save_rosbag(self):
        cmd = "rosrun mohou_ros save_rosbag.py -pn {} -t 8".format(self.project_name)
        n_episode = 3
        for _ in range(n_episode):
            self._run_command(cmd)

        rosbag_paths = get_rosbag_paths(self.project_path)
        assert len(rosbag_paths) == n_episode

        config = Config.from_project_path(self.project_path)
        topic_list = config.topics.rosbag_topic_list
        rosbag_path = rosbag_paths[0]
        bag = Bag(str(rosbag_path))
        seqs = bag_to_seqs(bag, topic_list)
        bag.close()

        for seq in seqs:
            assert len(seq) > 1

        obtained_topic_names = set([seq.topic_name for seq in seqs])
        assert_equal_with_message(obtained_topic_names, set(topic_list), "topic set in rosbag")

    def test_pipeline(self):
        self._test_save_home_position()
        self._test_save_rosbag()


if __name__ == "__main__":
    rospy.init_node("test_data_collection")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
