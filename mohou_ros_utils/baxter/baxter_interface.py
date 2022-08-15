from pathlib import Path
from typing import Tuple

import control_msgs.msg
import rospkg
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from skrobot.model import RobotModel
from skrobot.model.robot_model import RobotModel
from skrobot.models.urdf import RobotModelFromURDF


class BaxterROSRobotInterface(ROSRobotInterfaceBase):
    def __init__(self, robot: RobotModel):
        super(BaxterROSRobotInterface, self).__init__(
            robot, default_controller="default_controller", joint_states_topic="/robot/joint_states"
        )

    @property
    def rarm_controller(self):
        joint_names = [
            "right_s0",
            "right_s1",
            "right_e0",
            "right_e1",
            "right_w0",
            "right_w1",
            "right_w2",
        ]
        return dict(
            controller_type="rarm_controller",
            controller_action="/robot/limb/right/follow_joint_trajectory",
            controller_state="/robot/limb/right/state",
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=joint_names,
        )

    @property
    def larm_controller(self):
        joint_names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
        return dict(
            controller_type="larm_controller",
            controller_action="/robot/limb/left/follow_joint_trajectory",
            controller_state="/robot/limb/left/state",
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=joint_names,
        )

    def default_controller(self):
        return [self.rarm_controller, self.larm_controller]


def baxter_init() -> Tuple[RobotModel, ROSRobotInterfaceBase]:
    rospack = rospkg.RosPack()
    model_urdf_path = Path(rospack.get_path("baxter_description"))
    baxter_urdf_path = model_urdf_path / "urdf" / "baxter.urdf"
    robot_model = RobotModelFromURDF(urdf_file=str(baxter_urdf_path))

    ri = BaxterROSRobotInterface(robot_model)
    return robot_model, ri
