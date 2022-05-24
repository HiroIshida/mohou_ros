import os
import time

from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.config import Config


def count_rosbag_file(config: Config) -> int:
    rosbag_dir = get_rosbag_dir(config.project_name)
    counter = 0
    for filename in os.listdir(rosbag_dir):
        _, ext = os.path.splitext(filename)
        if ext == '.bag':
            counter += 1
    return counter


def get_latest_rosbag_filename(config: Config) -> str:
    rosbag_dir = get_rosbag_dir(config.project_name)
    filename_sorted = sorted(os.listdir(rosbag_dir))
    return os.path.join(rosbag_dir, filename_sorted[-1])


def get_rosbag_filename(config: Config, postfix: str):
    rosbag_dir = get_rosbag_dir(config.project_name)
    filename = os.path.join(rosbag_dir, 'train-episode-{0}.bag'.format(postfix))
    return filename


def create_rosbag_command(config: Config):
    postfix = time.strftime("%Y%m%d%H%M%S")
    filename = get_rosbag_filename(config, postfix)
    cmd_rosbag = ['rosbag', 'record']
    topic_list = config.topics.rosbag_topic_list
    cmd_rosbag.extend(topic_list + ['/tf'])
    cmd_rosbag.extend(['--output-name', filename])
    print('subprocess cmd: {}'.format(cmd_rosbag))
    return cmd_rosbag
