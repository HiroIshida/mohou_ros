#!/usr/bin/env python3
import argparse
import os
import yaml
import time

import rospy
import rospkg
from sensor_msgs.msg import JointState

from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_home_position_file


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))

    args = parser.parse_args()
    config_file = args.config
    config = Config.from_yaml_file(config_file)

    rospy.init_node('save_home_position')

    data = {'msg': None}

    def callback(msg):
        data['msg'] = msg

    av_topic_name = config.topics.av_topic_config.name
    rospy.Subscriber(av_topic_name, JointState, callback=callback)
    time.sleep(2.0)

    assert data['msg'] is not None
    msg: JointState = data['msg']
    name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
    joint_indices = [name_idx_map[name] for name in msg.name]
    joint_angle_map = {name: msg.position[name_idx_map[name]] for name in msg.name}

    with open(get_home_position_file(config.project), 'w') as f:
        yaml.dump(joint_angle_map, f)
