#!/usr/bin/env python3
import time

import rospy
import skrobot
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore


class NoTrosoPR2(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.rarm_controller, self.larm_controller, self.head_controller]


robot_model = skrobot.models.PR2()
ri = NoTrosoPR2(robot_model)
robot_model.init_pose()
ri.wait_interpolation()

while not rospy.is_shutdown():
    robot_model.init_pose()
    ri.angle_vector(robot_model.angle_vector(), time=2.0, time_scale=1.0)
    ri.move_gripper("rarm", 0.1, effort=25, wait=False)
    ri.move_gripper("larm", 0.1, effort=25, wait=False)
    time.sleep(2)

    robot_model.reset_manip_pose()
    ri.angle_vector(robot_model.angle_vector(), time=2.0, time_scale=1.0)
    ri.move_gripper("rarm", 0.0, effort=25, wait=False)
    ri.move_gripper("larm", 0.0, effort=25, wait=False)
    time.sleep(2)
