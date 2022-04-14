#!/usr/bin/env python3
from typing import Optional, List, Dict
import argparse
import os
import time
from skrobot.model import Joint
from skrobot.models import PR2
from skrobot.interfaces.ros import PR2ROSRobotInterface  # type: ignore
import rospy
import threading
import rospkg
from pr2_mechanism_msgs.srv import SwitchController
from pr2_mechanism_msgs.srv import ListControllers, ListControllersResponse

from mohou_ros_utils.config import Config

from params import larm_joint_names, rarm_joint_names
from params import larm_controller_name, rarm_controller_name, all_controller_names


def get_controller_states() -> Dict[str, bool]:
    sp = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
    resp: ListControllersResponse = sp()
    return {cont: (state == 'running') for (cont, state) in zip(resp.controllers, resp.state)}


def switch_controller(controllers: List[str], start: bool):
    loose_controllers = [con + '_loose' for con in controllers]
    sp = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

    if start:
        resp = sp(start_controllers=loose_controllers, stop_controllers=controllers)
    else:
        resp = sp(start_controllers=controllers, stop_controllers=loose_controllers, strictness=2)
    print('controller service response: {}'.format(resp))
    return resp


class Mannequin(object):
    robot: PR2
    ri: PR2ROSRobotInterface
    config: Config
    controller_names: List[str]
    larm_joints: List[Joint]
    rarm_joints: List[Joint]
    thread: Optional[threading.Thread]
    enable_mirror: bool

    def __init__(self, config: Config, loose_larm=True, loose_rarm=True, mirror=False):
        if not mirror:
            assert loose_larm or loose_rarm
        if mirror:
            loose_larm = True
            loose_rarm = False

        robot = PR2()

        controller_names = []
        if loose_larm:
            controller_names.append(larm_controller_name)
        if loose_rarm:
            controller_names.append(rarm_controller_name)

        self.robot = robot
        self.ri = PR2ROSRobotInterface(robot)
        self.config = config
        self.controller_names = controller_names
        self.larm_joints = [robot.__dict__[n] for n in larm_joint_names]
        self.rarm_joints = [robot.__dict__[n] for n in rarm_joint_names]
        self.is_thread_active = False
        self.thread = None
        self.enable_mirror = mirror

        time.sleep(3)
        self.reset_mannequin()

    def start_mannequine(self):
        switch_controller(self.controller_names, True)

    def stop_mannequine(self):
        switch_controller(all_controller_names, False)
        state_dict = get_controller_states()
        for cont in all_controller_names:
            assert state_dict[cont]

    def reset_mannequin(self):
        self.stop_mannequine()

        self.is_thread_active = False
        if self.thread is not None:
            self.thread.join()

        if self.config.home_position is None:
            return

        for joint_name in self.config.home_position.keys():
            angle = self.config.home_position[joint_name]
            self.robot.__dict__[joint_name].joint_angle(angle)
        self.ri.angle_vector(self.robot.angle_vector(), time=2.0, time_scale=1.0)
        print("resetting pose...")
        time.sleep(3.5)

    def mirror(self, time):

        if not self.enable_mirror:
            return

        self.robot.angle_vector(self.ri.angle_vector())
        langles = [j.joint_angle() for j in self.larm_joints]
        for j, a in zip(self.rarm_joints, langles):
            reverse_joints = ["r_upper_arm_roll_joint", "r_forearm_roll_joint", "r_wrist_roll_joint"]
            if j.name in reverse_joints:
                j.joint_angle(-a)
            elif j.name == "r_shoulder_pan_joint":
                j.joint_angle(-a + 1.0)
            else:
                j.joint_angle(a)
        self.ri.angle_vector(self.robot.angle_vector(), time=time, time_scale=1.0)

    def start(self):
        self.start_mannequine()
        self.is_thread_active = True

        def mirror_while():
            r = rospy.Rate(10)
            self.mirror(3.0)
            while self.is_thread_active:
                self.mirror(0.5)
                r.sleep()

        self.thread = threading.Thread(target=mirror_while)
        self.thread.daemon = True
        self.thread.start()

    def terminate(self):
        self.is_thread_active = False
        print("deactivate...")
        if self.thread is not None:
            self.thread.join()
        self.stop_mannequine()
        print("mannequin stopped")


if __name__ == '__main__':
    config_dir = os.path.join(rospkg.RosPack().get_path('mohou_ros'), 'configs')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_dir, 'pr2_rarm.yaml'))
    parser.add_argument('--rarm', action='store_true', help='loose rarm')
    parser.add_argument('--larm', action='store_true', help='loose larm')
    parser.add_argument('--mirror', action='store_true', help='mirror mode')

    args = parser.parse_args()
    loose_larm = args.larm
    loose_rarm = args.rarm
    mirror = args.mirror
    config_file = args.config
    config = Config.from_yaml_file(config_file)

    dic = get_controller_states()
    print(dic)

    mq = Mannequin(config, loose_larm=loose_larm, loose_rarm=loose_rarm, mirror=mirror)
    mq.start()

    while True:
        print('e: end mannequin, r: reset pose')
        key = input()
        if key == 'e':
            mq.terminate()
            break
        if key == 'r':
            mq.reset_mannequin()
            mq.start()
