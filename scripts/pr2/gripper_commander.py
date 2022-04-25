#!/usr/bin/env python3
import argparse
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-arm', type=str, default='rarm', help='which arm?')
    parser.add_argument('-open', type=float, default=0.04, help='max gripper position')
    parser.add_argument('-close', type=float, default=0.00, help='max gripper position')
    args = parser.parse_args()
    pos_open = args.open
    pos_close = args.close
    arm = args.arm

    assert arm in ['rarm', 'larm']

    if arm == 'rarm':
        topic_name = "/r_gripper_controller/gripper_action"
    else:
        topic_name = "/l_gripper_controller/gripper_action"

    rospy.init_node('gripper_commander')
    client = actionlib.SimpleActionClient(topic_name, Pr2GripperCommandAction)
    suc = client.wait_for_server()

    def create_goal(pos: float) -> Pr2GripperCommandActionGoal:
        goal = Pr2GripperCommandActionGoal()
        goal.goal.command.position = pos
        goal.goal.command.max_effort = 25
        return goal

    while True:
        print('o: open gripper, c: close gripper, e: end')
        key = input()
        if key == 'e':
            break
        elif key == 'o':
            g = create_goal(pos_open)
            client.send_goal(g.goal)
        elif key == 'c':
            g = create_goal(pos_close)
            client.send_goal(g.goal)
        else:
            pass
