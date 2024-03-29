#!/usr/bin/env python3
import argparse
import os
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import genpy
import numpy as np
import rosbag
import rospkg
from mohou.file import get_project_path
from mohou.types import (
    AngleVector,
    ElementSequence,
    EpisodeBundle,
    EpisodeData,
    MetaData,
    TimeStampSequence,
)

from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import MessageConverterCollection
from mohou_ros_utils.interpolator import (
    AllSameInterpolationRule,
    NearestNeighbourMessageInterpolator,
)
from mohou_ros_utils.rosbag import bag_to_synced_seqs
from mohou_ros_utils.script_utils import get_rosbag_paths
from mohou_ros_utils.types import TimeStampedSequence


def seqs_to_episodedata(
    seqs: List[TimeStampedSequence], config: Config, bagname: str
) -> EpisodeData:
    conv = MessageConverterCollection.from_config(config)

    mohou_elem_seqs = []
    for seq in seqs:
        assert seq.topic_name is not None
        if seq.topic_name not in config.topics.use_topic_list:
            continue

        elem_type = config.topics.get_by_topic_name(seq.topic_name).mohou_type
        elem_list = []
        for obj in seq.object_list:
            # TODO: to support mutli message to mohou type conversion
            # we must slice the time sequence
            assert isinstance(obj, genpy.Message)
            elem_list.append(conv.apply(obj, elem_type))
        elem_seq = ElementSequence(elem_list)
        mohou_elem_seqs.append(elem_seq)

    time_stamps = TimeStampSequence(seqs[0].time_list)
    metadata = MetaData()
    metadata["rosbag"] = bagname
    return EpisodeData.from_seq_list(mohou_elem_seqs, timestamp_seq=time_stamps, metadata=metadata)


class RemoveInitPolicy(Enum):
    remove = "remove"  # remove all constant-initial-states
    donothing = "donothing"  # just do not alter data
    skip = "skip"  # if too long constant initial state found, just ignore such data


@dataclass
class StaticInitStateRemover:

    policy: RemoveInitPolicy = RemoveInitPolicy.skip
    threshold_coef: float = 0.03

    @classmethod
    def from_policy_name(cls, name: str) -> "StaticInitStateRemover":
        return cls(RemoveInitPolicy(name))

    @staticmethod
    def find_av_static_duration(edata: EpisodeData) -> int:
        av_seq = edata.get_sequence_by_type(AngleVector)
        static_duration = 0
        for i in range(len(av_seq) - 1):
            diff = np.linalg.norm(av_seq[i + 1].numpy() - av_seq[i].numpy())  # type: ignore
            if diff > 0.005:  # TODO(HiroIshida) change this using mohou's std value
                static_duration = i
                break
        return static_duration

    def has_too_long_static_av(self, episode: EpisodeData):
        av_seq = episode.get_sequence_by_type(AngleVector)
        static_state_len_threshold = len(av_seq) * self.threshold_coef
        duration = self.find_av_static_duration(episode)
        too_long_static_av_duration = duration > static_state_len_threshold
        return too_long_static_av_duration

    def remove_init(self, episode: EpisodeData) -> Optional[EpisodeData]:

        if self.policy == RemoveInitPolicy.donothing:
            print("just do not alter...")
            return episode

        if self.policy == RemoveInitPolicy.remove:
            static_duration = self.find_av_static_duration(episode)
            print("remove initial {} state".format(static_duration))
            start_index = max(static_duration - 1, 0)
            episode_new = episode[start_index:]
            return episode_new

        if self.policy == RemoveInitPolicy.skip:
            if self.has_too_long_static_av(episode):
                return None
            else:
                return episode

        assert False


def main(
    config: Config,
    hz: float,
    dump_gif: bool,
    remover: StaticInitStateRemover,
    n_untouch_episode: int,
    postfix: Optional[str] = None,
    compress: bool = False,
) -> None:
    rosbag_paths = get_rosbag_paths(config.project_path)
    assert (
        len(rosbag_paths) > 0
    ), "please check if rosbag files are put in '{project_path}/rosbag' with .bag extension"

    episode_data_list = []
    for rosbag_path in rosbag_paths:
        print("processing {}".format(rosbag_path))

        bagname = rosbag_path.name

        topic_name_list = config.topics.use_topic_list
        print("topic_list: {}".format(topic_name_list))

        rule = AllSameInterpolationRule(NearestNeighbourMessageInterpolator)
        bag = rosbag.Bag(str(rosbag_path))
        seqs = bag_to_synced_seqs(bag, 1.0 / hz, topic_names=topic_name_list, rule=rule)
        bag.close()

        episode = seqs_to_episodedata(seqs, config, bagname)
        episode_init_removed = remover.remove_init(episode)
        if episode_init_removed is None:
            continue
        episode_data_list.append(episode_init_removed)

        if dump_gif:
            gif_dir_path = config.project_path / "train_data_gifs"
            gif_dir_path.mkdir(exist_ok=True)
            if postfix is None:
                gif_file_path = gif_dir_path / "{}.gif".format(bagname)
            else:
                gif_file_path = gif_dir_path / "{}-{}.gif".format(bagname, postfix)
            episode_init_removed.save_debug_gif(str(gif_file_path), fps=20)

    extra_info: MetaData = MetaData(
        {"hz": hz, "remove_init_policy": remover.policy.value, "compress": compress}
    )
    bundle = EpisodeBundle.from_episodes(
        episode_data_list, meta_data=extra_info, n_untouch_episode=n_untouch_episode
    )
    bundle.dump(config.project_path, postfix, compress=compress)
    bundle.plot_vector_histories(AngleVector, config.project_path, hz=hz, postfix=postfix)


if __name__ == "__main__":
    config_dir = os.path.join(rospkg.RosPack().get_path("mohou_ros"), "configs")
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-hz", type=float, default=5.0)
    parser.add_argument(
        "-remove_policy",
        type=str,
        default="skip",
        help="remove init policy when too long initial static angle vector found",
    )
    parser.add_argument("-postfix", type=str, default="", help="bundle postfix")
    parser.add_argument("--gif", action="store_true", help="dump gifs for debugging")
    parser.add_argument(
        "--compress", action="store_true", help="compress episode when dumping bundle"
    )
    parser.add_argument("-untouch", type=int, default=5, help="num of untouch episode")

    args = parser.parse_args()

    project_name: Optional[str] = args.pn
    hz: float = args.hz
    dump_gif: bool = args.gif
    n_untouch: int = args.untouch
    compress: bool = args.compress

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    postfix = None if args.postfix == "" else args.postfix
    remover = StaticInitStateRemover.from_policy_name(args.remove_policy)
    main(config, hz, dump_gif, remover, n_untouch, postfix, compress)
