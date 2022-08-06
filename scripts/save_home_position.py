#!/usr/bin/env python3
import argparse
import time
from typing import Optional

import rospy
import yaml
from mohou.file import get_project_path
from mohou.types import AngleVector
from sensor_msgs.msg import JointState

from mohou_ros_utils.config import Config
from mohou_ros_utils.file import RelativeName, get_subpath

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    args = parser.parse_args()

    project_name: Optional[str] = args.pn
    project_path = get_project_path(project_name)

    config = Config.from_project_path(project_path)

    rospy.init_node("save_home_position")

    data = {"msg": None}

    def callback(msg):
        data["msg"] = msg

    av_config = config.topics.get_by_mohou_type(AngleVector)
    av_topic_name_list = av_config.topic_name_list
    assert (
        len(av_topic_name_list) == 1
    ), "this impl assumes that AngleVector is extracted only from JointState"
    rospy.Subscriber(av_topic_name_list[0], JointState, callback=callback)
    time.sleep(2.0)

    assert data["msg"] is not None
    msg: JointState = data["msg"]
    name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
    joint_indices = [name_idx_map[name] for name in msg.name]
    joint_angle_map = {name: msg.position[name_idx_map[name]] for name in msg.name}

    project_path.mkdir(exist_ok=True)
    home_position_path = get_subpath(project_path, RelativeName.home_position)
    with home_position_path.open(mode="w") as f:
        yaml.dump(joint_angle_map, f)
