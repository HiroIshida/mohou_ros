import os
import time

from mohou_ros_utils.file import get_rosbag_dir
from mohou_ros_utils.config import Config


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
