#!/usr/bin/env python3
import argparse
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional

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
    RGBImage,
    TimeStampSequence,
)
from moviepy.editor import ImageSequenceClip

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.conversion import VersatileConverter
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.interpolator import (
    AllSameInterpolationRule,
    NearestNeighbourMessageInterpolator,
)
from mohou_ros_utils.rosbag import bag_to_synced_seqs
from mohou_ros_utils.types import TimeStampedSequence


def seqs_to_episodedata(
    seqs: List[TimeStampedSequence], config: Config, bagname: str
) -> EpisodeData:
    vconv = VersatileConverter.from_config(config)

    mohou_elem_seqs = []
    for seq in seqs:
        assert seq.topic_name is not None
        if seq.topic_name not in config.topics.use_topic_list:
            continue

        elem_type = config.topics.get_by_topic_name(seq.topic_name).mohou_type
        elem_seq = ElementSequence([vconv.converters[elem_type](e) for e in seq.object_list])
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
):
    rosbag_dir_path = Path(get_rosbag_dir(config.project_name))
    rosbag_paths = [p.expanduser() for p in rosbag_dir_path.iterdir() if p.name.endswith(".bag")]
    assert (
        len(rosbag_paths) > 0
    ), "please check if rosbag files are put in '{project_path}/rosbag' with .bag extension"

    project_path = get_project_path(config.project_name)

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
            gif_dir_path = project_path / "train_data_gifs"
            gif_dir_path.mkdir(exist_ok=True)
            fps = 20
            images = [rgb.numpy() for rgb in episode_init_removed.get_sequence_by_type(RGBImage)]
            clip = ImageSequenceClip(images, fps=fps)

            if postfix is None:
                gif_file_path = gif_dir_path / "{}.gif".format(bagname)
            else:
                gif_file_path = gif_dir_path / "{}-{}.gif".format(bagname, postfix)
            clip.write_gif(str(gif_file_path), fps=fps)

    extra_info: MetaData = MetaData({"hz": hz, "remove_init_policy": remover.policy.value})
    bundle = EpisodeBundle.from_episodes(
        episode_data_list, meta_data=extra_info, n_untouch_episode=n_untouch_episode
    )
    bundle.dump(project_path, postfix)
    bundle.plot_vector_histories(AngleVector, project_path, hz=hz, postfix=postfix)


if __name__ == "__main__":
    config_dir = os.path.join(rospkg.RosPack().get_path("mohou_ros"), "configs")
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    parser.add_argument("-hz", type=float, default=5.0)
    parser.add_argument(
        "-remove_policy",
        type=str,
        default="skip",
        help="remove init policy when too long initial static angle vector found",
    )
    parser.add_argument("-postfix", type=str, default="", help="bundle postfix")
    parser.add_argument("--gif", action="store_true", help="dump gifs for debugging")
    parser.add_argument("-untouch", type=int, default=5, help="num of untouch episode")

    args = parser.parse_args()
    config = Config.from_project_name(args.pn)
    hz: float = args.hz
    dump_gif: bool = args.gif
    n_untouch: int = args.untouch

    postfix = None if args.postfix == "" else args.postfix
    remover = StaticInitStateRemover.from_policy_name(args.remove_policy)
    main(config, hz, dump_gif, remover, n_untouch, postfix)
