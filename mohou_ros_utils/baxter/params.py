from typing import List

larm_joint_names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]

rarm_joint_names = [
    "right_s0",
    "right_s1",
    "right_e0",
    "right_e1",
    "right_w0",
    "right_w1",
    "right_w2",
]

larm_controller_name = "l_arm_controller"

rarm_controller_name = "r_arm_controller"

all_controller_names = [larm_controller_name, rarm_controller_name]


class BaxterRarmProperty:
    @property
    def control_joint_names(self) -> List[str]:
        return rarm_joint_names

    @property
    def end_effector_name(self) -> str:
        return "right_hand"


class BaxterLarmProperty:
    @property
    def control_joint_names(self) -> List[str]:
        return larm_joint_names

    @property
    def end_effector_name(self) -> str:
        return "left_hand"
