#!/usr/bin/env python3
import argparse
import os
import subprocess
import signal
import time

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.config import Config


def get_rosbag_filename(config: Config, postfix: str, auxiliary: bool):
    rosbag_dir = get_rosbag_dir(config.project_name)
    if auxiliary:
        filename = os.path.join(rosbag_dir, 'auxiliary-episode-{0}.bag'.format(postfix))
    else:
        filename = os.path.join(rosbag_dir, 'train-episode-{0}.bag'.format(postfix))
    return filename


def create_rosbag_command(config: Config, postfix: str, auxiliary: bool):
    filename = get_rosbag_filename(config, postfix, auxiliary)
    cmd_rosbag = ['rosbag', 'record']
    topic_list = config.topics.auxiliary_topic_list if auxiliary else config.topics.rosbag_topic_list
    cmd_rosbag.extend(topic_list + ['/tf'])
    cmd_rosbag.extend(['--output-name', filename])
    print('subprocess cmd: {}'.format(cmd_rosbag))
    return cmd_rosbag


"""Deprecated! because not working well with regacy environment e.g. inside PR2
def dump_debug_image(config: Config, postfix: str):
    rosbag_filename = get_rosbag_filename(config, postfix)
    bag = rosbag.Bag(rosbag_filename)
    topic_names = [config.topics.rgb_topic, config.topics.depth_topic]
    seqs = bag_to_seqs(bag, topic_names)
    bag.close()

    vconv = VersatileConverter.from_config(config)

    for seq in seqs:
        image_type: Type[ElementBase]
        if seq.topic_name == config.topics.depth_topic:
            image_type = DepthImage
        elif seq.topic_name == config.topics.rgb_topic:
            image_type = RGBImage
        else:
            continue
        images = [vconv.converters[image_type](msg).to_rgb() for msg in seq.object_list]

        fps = 20
        gif_filename = os.path.splitext(rosbag_filename)[0] + '-{}.gif'.format(image_type.__name__)
        clip = ImageSequenceClip(images, fps=fps)
        clip.write_gif(gif_filename, fps=fps)
"""


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-pn', type=str, default=_default_project_name, help='project name')
    parser.add_argument('--aux', action='store_true', help='rosbag for auxiliary data')

    args = parser.parse_args()
    auxiliary = args.aux
    config = Config.from_project_name(args.pn)

    postfix = time.strftime("%Y%m%d%H%M%S")
    cmd = create_rosbag_command(config, postfix, auxiliary)
    p = subprocess.Popen(cmd)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        os.kill(p.pid, signal.SIGKILL)
        time.sleep(1)  # a workaround
