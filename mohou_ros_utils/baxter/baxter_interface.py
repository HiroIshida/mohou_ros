from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
import control_msgs.msg
from pathlib import Path
from skrobot.model import RobotModel
from skrobot.model.robot_model import RobotModel
from skrobot.models.urdf import RobotModelFromURDF
from typing import Tuple
import rospkg


class BaxterROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, robot: RobotModel):
        super(BaxterROSRobotInterface, self).__init__(
            robot,
            default_controller="default_controller",
            joint_states_topic="/robot/joint_states")

    @property
    def rarm_controller(self):
        return dict(
            controller_type='rarm_controller',
            controller_action='/robot/limb/right/follow_joint_trajectory',
            controller_state='/robot/limb/left/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[j.name for j in self.robot.rarm.joint_list],
        )

    def default_controller(self):
        return [self.rarm_controller]


def baxter_init() -> Tuple[RobotModel, ROSRobotInterfaceBase]:
    rospack = rospkg.RosPack()
    model_urdf_path = Path(rospack.get_path("baxter_description"))
    baxter_urdf_path = model_urdf_path / "urdf" / "baxter.urdf"
    robot_model = RobotModelFromURDF(urdf_file=str(baxter_urdf_path))

    ri = BaxterROSRobotInterface(robot_model)
    return robot_model, ri
