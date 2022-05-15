from typing import List, Dict
import pprint
import rospy
from pr2_mechanism_msgs.srv import SwitchController
from pr2_mechanism_msgs.srv import ListControllers, ListControllersResponse

from mohou_ros_utils.pr2.params import larm_controller_name, rarm_controller_name


def get_controller_states() -> Dict[str, bool]:
    sp = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
    resp: ListControllersResponse = sp()
    return {cont: (state == 'running') for (cont, state) in zip(resp.controllers, resp.state)}


def check_pr2_is_executable() -> None:
    states = get_controller_states()
    if not (states[larm_controller_name] and states[rarm_controller_name]):
        rospy.logwarn('you must turn on pr2 both rarm and larm controllers')
        rospy.logwarn('Please check the controler state below ↓')
        rospy.logwarn(pprint.pformat(states))
        raise RuntimeError


def switch_controller(controllers: List[str], start: bool):
    loose_controllers = [con + '_loose' for con in controllers]
    sp = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

    if start:
        start_controllers = loose_controllers
        stop_controllers = controllers
    else:
        start_controllers = controllers
        stop_controllers = loose_controllers

    rospy.loginfo('start: {}'.format(start_controllers))
    rospy.loginfo('stop: {}'.format(stop_controllers))
    resp = sp(start_controllers=start_controllers, stop_controllers=stop_controllers)
    rospy.loginfo('controller service response: {}'.format(resp))
    rospy.loginfo('current controller status')
    states = get_controller_states()
    rospy.loginfo(pprint.pformat(states))
    return resp
