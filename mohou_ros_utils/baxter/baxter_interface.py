from pathlib import Path
from typing import Tuple

import control_msgs.msg
import rospkg
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from skrobot.model import RobotModel
from skrobot.models.urdf import RobotModelFromURDF

from mohou_ros_utils.baxter.params import larm_joint_names, rarm_joint_names


class BaxterROSRobotInterface(ROSRobotInterfaceBase):
    def __init__(self, robot: RobotModel):
        super(BaxterROSRobotInterface, self).__init__(
            robot, default_controller="default_controller", joint_states_topic="/robot/joint_states"
        )

    @property
    def rarm_controller(self):
        joint_names = larm_joint_names
        return dict(
            controller_type="rarm_controller",
            controller_action="/robot/limb/right/follow_joint_trajectory",
            controller_state="/robot/limb/right/state",
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=joint_names,
        )

    @property
    def larm_controller(self):
        joint_names = rarm_joint_names
        return dict(
            controller_type="larm_controller",
            controller_action="/robot/limb/left/follow_joint_trajectory",
            controller_state="/robot/limb/left/state",
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=joint_names,
        )

    def default_controller(self):
        return [self.rarm_controller, self.larm_controller]


class BaxterRarmInterface(BaxterROSRobotInterface):
    def default_controller(self):
        return [self.rarm_controller]


class BaxterLarmInterface(BaxterROSRobotInterface):
    def default_controller(self):
        return [self.larm_controller]


def baxter_init() -> Tuple[RobotModel, ROSRobotInterfaceBase]:
    rospack = rospkg.RosPack()
    model_urdf_path = Path(rospack.get_path("baxter_description"))
    baxter_urdf_path = model_urdf_path / "urdf" / "baxter.urdf"
    robot_model = RobotModelFromURDF(urdf_file=str(baxter_urdf_path))

    ri = BaxterROSRobotInterface(robot_model)
    return robot_model, ri
