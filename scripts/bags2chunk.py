#!/usr/bin/env python3
import argparse
import os
import numpy as np
import rosbag
import rospkg
from typing import List
from moviepy.editor import ImageSequenceClip
from mohou.file import get_project_dir
from mohou.types import RGBImage, AngleVector
from mohou.types import MultiEpisodeChunk, EpisodeData, ElementSequence

from mohou_ros_utils.types import TimeStampedSequence
from mohou_ros_utils.file import get_rosbag_dir, create_if_not_exist
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
        if seq.topic_name not in config.topics.use_topic_list:
            continue

        elem_type = config.topics.get_by_topic_name(seq.topic_name).mohou_type
        elem_seq = ElementSequence([vconv.converters[elem_type](e) for e in seq.object_list])
        mohou_elem_seqs.append(elem_seq)
    return EpisodeData.from_seq_list(mohou_elem_seqs)


def has_too_long_static_av(edata: EpisodeData, coef: float = 0.1):
    av_seq = edata.get_sequence_by_type(AngleVector)
    static_state_len_threshold = len(av_seq) * coef
    indices_static = None
    for i in range(len(av_seq) - 1):
        diff = np.linalg.norm(av_seq[i + 1].numpy() - av_seq[i].numpy())  # type: ignore
        if diff > 0.005:  # TODO(HiroIshida) change this using mohou's std value
            indices_static = i
            break
    if indices_static is None:
        return False
    return indices_static > static_state_len_threshold


def main(config: Config, dump_gif, auxiliary):
    rosbag_dir = get_rosbag_dir(config.project)
    episode_data_list = []
    for filename_ in os.listdir(rosbag_dir):
        print('processing {}'.format(filename_))
        _, ext = os.path.splitext(filename_)
        if ext != '.bag':
            print('skipped (invalid file extension)')
            continue

        if auxiliary:
            if not filename_.startswith('auxiliary'):
                continue
        else:
            if not filename_.startswith('train'):
                continue

        filename = os.path.join(rosbag_dir, filename_)

        rule = AllSameInterpolationRule(NearestNeighbourMessageInterpolator)
        bag = rosbag.Bag(filename)
        seqs = bag_to_synced_seqs(bag,
                                  1.0 / config.hz,
                                  topic_names=config.topics.use_topic_list,
                                  rule=rule)
        bag.close()

        episode_data = seqs_to_episodedata(seqs, config)
        if has_too_long_static_av(episode_data):
            print('skipped (strange too long initial static state found. skipped)')
            continue
        episode_data_list.append(episode_data)

    if auxiliary:
        chunk = MultiEpisodeChunk.from_data_list(episode_data_list, with_intact_data=False)
        chunk.dump_aux(config.project)
    else:
        chunk = MultiEpisodeChunk.from_data_list(episode_data_list)
        chunk.dump(config.project)

    if dump_gif:
        if auxiliary:
            gif_dir = os.path.join(get_project_dir(config.project), 'auxiliary_data_gifs')
        else:
            gif_dir = os.path.join(get_project_dir(config.project), 'train_data_gifs')
        create_if_not_exist(gif_dir)
        for i, episode_data in enumerate(chunk):
            fps = 20
            images = [rgb.numpy() for rgb in episode_data.get_sequence_by_type(RGBImage)]
            clip = ImageSequenceClip(images, fps=fps)

            gif_filename = os.path.join(gif_dir, '{}.gif'.format(i))
            clip.write_gif(gif_filename, fps=fps)


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))
    parser.add_argument('--aux', action='store_true', help='convert auxiliary data')
    parser.add_argument('--gif', action='store_true', help='dump gifs for debugging')

    args = parser.parse_args()
    config = Config.from_yaml_file(args.config)
    dump_gif = args.gif
    auxiliary = args.aux
    main(config, dump_gif, auxiliary)
