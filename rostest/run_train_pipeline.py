#!/usr/bin/env python3
import subprocess
import unittest
from pathlib import Path

import rospy
from mohou.file import get_project_path
from mohou.model.autoencoder import VariationalAutoEncoder
from mohou.model.lstm import LSTM
from mohou.trainer import TrainCache
from mohou.types import EpisodeBundle

import rostest
from mohou_ros_utils.script_utils import get_rosbag_paths


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

    def _test_data_collection_finsihed(self):
        # check if data collection rostest is already finished
        assert self.project_path.exists()
        rosbag_paths = get_rosbag_paths(self.project_path)
        assert len(rosbag_paths) > 2

    def _test_visualize_rosbag(self):
        cmd = "rosrun mohou_ros visualize_rosbag.py -pn {} -n 2 -ext mp4".format(self.project_name)
        self._run_command(cmd)

    def _test_tune_image_filter(self):
        cmd = "rosrun mohou_ros tune_image_filter.py -pn {} --testrun".format(self.project_name)
        self._run_command(cmd)

    def _test_bags2bundle(self):

        cmd = "rosrun mohou_ros bags2chunk.py -hz 10 -remove_policy donothing -pn {} -untouch 1 --gif".format(
            self.project_name
        )
        self._run_command(cmd)

        # check
        EpisodeBundle.load(self.project_path)
        gif_dir_path = self.project_path / "train_data_gifs"
        assert gif_dir_path.exists()
        assert len([p for p in gif_dir_path.iterdir() if p.name.endswith(".gif")]) > 0

        cmd = "rosrun mohou_ros bags2chunk.py -hz 20 -postfix autoencoder -remove_policy remove -pn {} -untouch 1 --gif".format(
            self.project_name
        )
        self._run_command(cmd)

        # check
        EpisodeBundle.load(self.project_path, postfix="autoencoder")
        assert len([p for p in gif_dir_path.iterdir() if p.name.endswith("autoencoder.gif")]) > 0

    def _test_train(self):
        cmd = "rosrun mohou_ros train.py -pn {} --test".format(self.project_name)
        self._run_command(cmd)

        project_path = get_project_path(self.project_name)

        # check train model is cached
        tcache_lstm_list = TrainCache.load_all(project_path, LSTM)
        assert len(tcache_lstm_list) > 0

        tcache_vae_list = TrainCache.load_all(project_path, VariationalAutoEncoder)
        assert len(tcache_vae_list) > 0

        # check history plot is saved
        train_history_dir_path = project_path / "train_history"
        png_list = [p for p in train_history_dir_path.iterdir() if str(p).endswith(".png")]
        assert len(png_list) == 2

        # check vae and lstm result gif is found
        vae_result_dir_path = project_path / "autoencoder_result"
        png_list = [p for p in vae_result_dir_path.iterdir() if str(p).endswith(".png")]
        assert len(png_list) > 0
        gif_list = [p for p in vae_result_dir_path.iterdir() if str(p).endswith(".gif")]
        assert len(gif_list) > 0

        lstm_result_dir_path = project_path / "lstm_result"
        png_list = [p for p in lstm_result_dir_path.iterdir() if str(p).endswith(".png")]
        assert len(png_list) > 0
        gif_list = [p for p in lstm_result_dir_path.iterdir() if str(p).endswith(".gif")]
        assert len(gif_list) > 0

    def test_pipeline(self):
        self._test_data_collection_finsihed()
        self._test_visualize_rosbag()
        self._test_tune_image_filter()
        self._test_bags2bundle()
        self._test_train()


if __name__ == "__main__":
    rospy.init_node("test_sample")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
