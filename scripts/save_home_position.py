#!/usr/bin/env python3
import argparse
import time

import rospy
import yaml
from mohou.types import AngleVector
from sensor_msgs.msg import JointState

from mohou_ros_utils import _default_project_name
from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_home_position_file

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, default=_default_project_name, help="project name")
    args = parser.parse_args()
    config = Config.from_project_name(args.pn)

    rospy.init_node("save_home_position")

    data = {"msg": None}

    def callback(msg):
        data["msg"] = msg

    av_topic_name = config.topics.get_by_mohou_type(AngleVector).name
    rospy.Subscriber(av_topic_name, JointState, callback=callback)
    time.sleep(2.0)

    assert data["msg"] is not None
    msg: JointState = data["msg"]
    name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
    joint_indices = [name_idx_map[name] for name in msg.name]
    joint_angle_map = {name: msg.position[name_idx_map[name]] for name in msg.name}

    with open(get_home_position_file(config.project_name), "w") as f:
        yaml.dump(joint_angle_map, f)
