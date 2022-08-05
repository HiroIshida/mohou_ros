#!/usr/bin/env python3
import time
import unittest

import numpy as np
import rospy
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
from skrobot.models import PR2

import rostest


class TestNode(unittest.TestCase):
    def test_pr2eus_commander(self):
        names = ["l_shoulder_lift_joint", "head_pan_joint", "torso_lift_joint"]
        ref_angles = np.array([0.3, 0.3, 0.25])

        model = PR2()
        ri = PR2ROSRobotInterface(model)

        ts = time.time()
        timeout = 240
        while True:
            time.sleep(0.5)
            ri.angle_vector()
            model.angle_vector(ri.angle_vector())
            angles = np.array([model.__dict__[jname].joint_angle() for jname in names])
            error = ref_angles - angles
            rospy.loginfo("desired - current: {}".format(error))
            if np.linalg.norm(error) < 0.03:
                return

            elapsed = time.time() - ts
            if elapsed > timeout:
                break

        assert False, "desired angle is not satisifed. controller seems not working"


if __name__ == "__main__":
    rospy.init_node("test_pr2eus_commaner")
    rostest.rosrun("mohou_ros", "test_node", TestNode)
