#!/usr/bin/env python3
import argparse
import os
import rosbag
import rospkg
from typing import List, Type
from mohou.types import RGBImage, DepthImage, ElementBase, AngleVector
from mohou.types import MultiEpisodeChunk, EpisodeData, ElementSequence

from mohou_ros_utils.types import TimeStampedSequence
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou_ros_utils.interpolator import AllSameInterpolationRule
from mohou_ros_utils.interpolator import NearestNeighbourMessageInterpolator
from mohou_ros_utils.rosbag import bag_to_synced_seqs


def seqs_to_episodedata(seqs: List[TimeStampedSequence], config: Config) -> EpisodeData:
    vconv = VersatileConverter.from_config(config)

    mohou_elem_seqs = []
    for seq in seqs:
        assert seq.topic_name is not None
        elem_type: Type[ElementBase]
        if seq.topic_name == config.topics.rgb_topic:
            elem_type = RGBImage
        elif seq.topic_name == config.topics.depth_topic:
            elem_type = DepthImage
        elif seq.topic_name == config.topics.av_topic:
            elem_type = AngleVector
        else:
            assert False
        elem_seq = ElementSequence([vconv.converters[elem_type](e) for e in seq.object_list])
        mohou_elem_seqs.append(elem_seq)
    return EpisodeData(tuple(mohou_elem_seqs))


def main(config: Config):
    rosbag_dir = get_rosbag_dir(config.project)
    episode_data_list = []
    for filename_ in os.listdir(rosbag_dir):
        filename = os.path.join(rosbag_dir, filename_)
        _, ext = os.path.splitext(filename)
        if ext != '.bag':
            continue

        rule = AllSameInterpolationRule(NearestNeighbourMessageInterpolator)
        bag = rosbag.Bag(filename)
        seqs = bag_to_synced_seqs(bag,
                                  1.0 / config.hz,
                                  topic_names=config.topics.topic_list,
                                  rule=rule)
        bag.close()

        episode_data = seqs_to_episodedata(seqs, config)
        episode_data_list.append(episode_data)
    chunk = MultiEpisodeChunk(episode_data_list)
    chunk.dump(config.project)


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))

    args = parser.parse_args()
    config = Config.from_yaml_file(args.config)
    main(config)
