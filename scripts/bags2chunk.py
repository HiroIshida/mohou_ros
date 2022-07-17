#!/usr/bin/env python3
import argparse
import os
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np
import rosbag
import rospkg
from mohou.file import get_project_path
from mohou.types import (
    AngleVector,
    ElementSequence,
    EpisodeData,
    MetaData,
    MultiEpisodeChunk,
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


class AmendPolicy(Enum):
    amend = "amend"  # remove all constant-initial-states
    donothing = "donothing"  # just do not alter data
    skip = "skip"  # if too long constant initial state found, just ignore such data


@dataclass
class StaticInitialStateAmender:

    policy: AmendPolicy = AmendPolicy.skip
    threshold_coef: float = 0.03

    @classmethod
    def from_policy_name(cls, name: str) -> "StaticInitialStateAmender":
        return cls(AmendPolicy(name))

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

    def amend(self, episode: EpisodeData) -> Optional[EpisodeData]:

        if self.policy == AmendPolicy.donothing:
            print("just do not alter...")
            return episode

        if self.policy == AmendPolicy.amend:
            static_duration = self.find_av_static_duration(episode)
            print("amending: remove initial {} state".format(static_duration))
            start_index = max(static_duration - 1, 0)
            episode_new = episode[start_index:]
            return episode_new

        if self.policy == AmendPolicy.skip:
            if self.has_too_long_static_av(episode):
                return None
            else:
                return episode

        assert False


def main(
    config: Config,
    hz: float,
    dump_gif: bool,
    amender: StaticInitialStateAmender,
    postfix: Optional[str] = None,
):
    rosbag_dir = get_rosbag_dir(config.project_name)
    episode_data_list = []
    filenames = os.listdir(rosbag_dir)
    assert len(filenames) > 0, "probably there is no rosbag files under the project directory"

    for filename_ in filenames:
        print("processing {}".format(filename_))
        _, ext = os.path.splitext(filename_)
        if ext != ".bag":
            print("skipped (invalid file extension)")
            continue

        filename = os.path.join(rosbag_dir, filename_)

        topic_name_list = config.topics.use_topic_list
        print("topic_list: {}".format(topic_name_list))

        rule = AllSameInterpolationRule(NearestNeighbourMessageInterpolator)
        bag = rosbag.Bag(filename)
        seqs = bag_to_synced_seqs(bag, 1.0 / hz, topic_names=topic_name_list, rule=rule)
        bag.close()

        episode = seqs_to_episodedata(seqs, config, filename_)
        episode_amended = amender.amend(episode)
        if episode_amended is None:
            continue
        episode_data_list.append(episode_amended)

        if dump_gif:
            gif_dir_path = get_project_path(config.project_name) / "train_data_gifs"
            gif_dir_path.mkdir(exist_ok=True)
            fps = 20
            images = [rgb.numpy() for rgb in episode_amended.get_sequence_by_type(RGBImage)]
            clip = ImageSequenceClip(images, fps=fps)

            if postfix is None:
                gif_file_path = gif_dir_path / "{}.gif".format(filename_)
            else:
                gif_file_path = gif_dir_path / "{}-{}.gif".format(filename_, postfix)
            clip.write_gif(str(gif_file_path), fps=fps)

    extra_info: MetaData = MetaData({"hz": hz})
    chunk = MultiEpisodeChunk.from_data_list(episode_data_list, extra_info=extra_info)
    chunk.dump(config.project_name, postfix)
    chunk.plot_vector_histories(AngleVector, config.project_name, hz=hz, postfix=postfix)


if __name__ == "__main__":
    config_dir = os.path.join(rospkg.RosPack().get_path("mohou_ros"), "configs")
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    parser.add_argument("-hz", type=float, default=5.0)
    parser.add_argument(
        "-amend_policy",
        type=str,
        default="skip",
        help="amend policy when too long initial static angle vector found",
    )
    parser.add_argument("-postfix", type=str, default="", help="chunk postfix")
    parser.add_argument("--gif", action="store_true", help="dump gifs for debugging")

    args = parser.parse_args()
    config = Config.from_project_name(args.pn)
    hz = args.hz
    dump_gif = args.gif

    postfix = None if args.postfix == "" else args.postfix
    amender = StaticInitialStateAmender.from_policy_name(args.amend_policy)
    main(config, hz, dump_gif, amender, postfix)
