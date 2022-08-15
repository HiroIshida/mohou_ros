from skrobot.interfaces.ros.pr2 import PR2ROSRobotInterface


class PR2RarmInterface(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.rarm_controller, self.torso_controller, self.head_controller]


class PR2LarmInterface(PR2ROSRobotInterface):
    def default_controller(self):
        return [self.larm_controller, self.torso_controller, self.head_controller]
