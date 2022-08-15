#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, ClassVar, Optional

import rospy
from mohou.file import get_project_path
from skrobot.models import PR2
from sound_play.libsoundplay import SoundClient

from mohou_ros_utils.config import Config
from mohou_ros_utils.script_utils import (
    count_rosbag_file,
    create_rosbag_command,
    get_latest_rosbag_filename,
    get_rosbag_filepath,
)
from mohou_ros_utils.vive_controller.robot_interface import (
    SkrobotPR2LarmController,
    SkrobotPR2RarmController,
)
from mohou_ros_utils.vive_controller.utils import detect_controller_ids
from mohou_ros_utils.vive_controller.vive_base import (
    JoyDataManager,
    ViveRobotController,
)


@dataclass
class RosbagManager:
    config: Config
    sound_client: SoundClient
    closure_stop: Optional[Callable] = None

    @property
    def is_running(self) -> bool:
        return self.closure_stop is not None

    def start(self) -> None:
        assert not self.is_running
        path = get_rosbag_filepath(self.config.project_path, time.strftime("%Y%m%d%H%M%S"))
        cmd = create_rosbag_command(path, config)
        p = subprocess.Popen(cmd)
        rospy.loginfo(p)
        share = {"is_running": True}

        def closure_stop():
            share["is_running"] = False

        self.closure_stop = closure_stop

        class Observer(threading.Thread):
            def run(self):
                while True:
                    time.sleep(0.5)
                    if not share["is_running"]:
                        rospy.loginfo("kill rosbag process")
                        os.kill(p.pid, signal.SIGTERM)
                        break

        self.sound_client.say("Start saving rosbag")
        thread = Observer()
        thread.start()

    def stop(self) -> None:
        assert self.is_running
        assert self.closure_stop is not None
        n_count = count_rosbag_file(self.config.project_path) + 1  # because we are gonna add one
        self.sound_client.say("Finish saving rosbag. Total number is {}".format(n_count))

        self.closure_stop()
        self.closure_stop = None

    def switch_state(self) -> None:
        rospy.loginfo("switch rosbag state")
        if self.is_running:
            self.stop()
        else:
            self.start()


class PR2RightArmViveController(ViveRobotController[SkrobotPR2RarmController]):
    rosbag_manager: RosbagManager
    log_prefix: ClassVar[str] = "Right"

    def __init__(self, controller_id: str, scale: float, config: Config):
        robot_con = SkrobotPR2RarmController(PR2())
        assert config.home_position is not None
        home_gripper_pos = config.home_position["r_gripper_joint"]
        super().__init__(controller_id, robot_con, scale, config.home_position, home_gripper_pos)

        rosbag_manager = RosbagManager(config, self.sound_client)
        self.rosbag_manager = rosbag_manager
        self.joy_manager.register_processor(
            JoyDataManager.Button.FRONT, rosbag_manager.switch_state
        )


class PR2LeftArmViveController(ViveRobotController[SkrobotPR2LarmController]):
    project_path: Path
    log_prefix: ClassVar[str] = "Left"

    def __init__(self, controller_id: str, scale: float, config: Config):
        robot_con = SkrobotPR2LarmController(PR2())
        assert config.home_position is not None
        home_gripper_pos = config.home_position["l_gripper_joint"]
        super().__init__(controller_id, robot_con, scale, config.home_position, home_gripper_pos)

        self.project_path = config.project_path
        self.joy_manager.register_processor(JoyDataManager.Button.FRONT, self.delete_latest_rosbag)

    def delete_latest_rosbag(self) -> None:
        latest_rosbag = get_latest_rosbag_filename(self.project_path)
        if latest_rosbag is None:
            message = "deleting rosbag failed because there is no rosbag"
            rospy.logwarn(message)
            self.sound_client.say(message)
        else:
            rospy.logwarn("delete rosbag file named {}".format(latest_rosbag))
            self.sound_client.say("delete latest rosbag")
            os.remove(latest_rosbag)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-scale", type=float, default=1.5, help="controller to real scaling")
    args, unknown = parser.parse_known_args()

    scale: float = args.scale
    project_name: Optional[str] = args.pn

    project_path = get_project_path(project_name)
    config = Config.from_project_path(project_path)

    while True:
        controller_ids = detect_controller_ids()
        print("waiting for vive controllers detected")
        if len(controller_ids) == 2:
            print("controllers: {} are detected".format(controller_ids))
            break

    rospy.init_node("pr2_vive_mohou")
    rarm_con = PR2RightArmViveController(controller_ids[0], scale, config)
    larm_con = PR2LeftArmViveController(controller_ids[1], scale, config)
    rarm_con.start()
    larm_con.start()
    rospy.spin()
