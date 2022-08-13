#!/usr/bin/env python3
import time

import rospy

from mohou_ros.msg import ControlCommand


def create_dummy_command() -> ControlCommand:
    joint_names = []
    joint_angles = []

    joint_names.append("l_shoulder_lift_joint")
    joint_angles.append(0.3)  # rad

    joint_names.append("head_pan_joint")
    joint_angles.append(0.3)  # rad

    joint_names.append("torso_lift_joint")
    joint_angles.append(0.25)  # m

    cmd = ControlCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.joint_names = joint_names
    cmd.angles = joint_angles
    cmd.rarm_gripper_angle = 0.045 # m
    cmd.larm_gripper_angle = 0.045 # m
    return cmd


if __name__ == "__main__":
    rospy.init_node("dummy_command_publisher")
    pub = rospy.Publisher("/mohou_control_command", ControlCommand, queue_size=3)

    while not rospy.is_shutdown():
        time.sleep(0.1)
        cmd = create_dummy_command()
        pub.publish(cmd)
