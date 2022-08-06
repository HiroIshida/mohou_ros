#!/usr/bin/env python3
import time
from math import pi, sin

import rospy

from mohou_ros.msg import ControlCommand


def create_dummy_command(time_step: int) -> ControlCommand:
    joint_names = []
    joint_angles = []

    T = 60
    joint_names.append("l_shoulder_lift_joint")
    joint_angles.append(0.8 * sin(pi * 2 * time_step / (2 * T)))  # rad

    joint_names.append("head_pan_joint")
    joint_angles.append(0.8 * sin(pi * 2 * time_step / (2 * T)))  # rad

    joint_names.append("torso_lift_joint")
    joint_angles.append(0.2 * sin(pi * 2 * time_step / (2 * T)) + 0.2)  # m

    cmd = ControlCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.joint_names = joint_names
    cmd.angles = joint_angles
    return cmd


if __name__ == "__main__":
    rospy.init_node("dummy_command_publisher")
    pub = rospy.Publisher("/mohou_control_command", ControlCommand, queue_size=3)

    time_step = 0
    while not rospy.is_shutdown():
        time.sleep(0.1)
        cmd = create_dummy_command(time_step)
        pub.publish(cmd)
        time_step += 1
