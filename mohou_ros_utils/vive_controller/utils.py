import re
import subprocess
from typing import List

import rospy


def detect_controller_ids() -> List[str]:
    try:
        ret = subprocess.check_output("rostopic list|grep controller_LHR|grep joy", shell=True)
    except subprocess.CalledProcessError:
        rospy.logwarn("It seems that no vive controller topic is published...")
        assert False

    topics = ret.decode("UTF-8").split("\n")
    name_list = []
    for topic in topics:
        m = re.match(r"/controller_LHR_(\w*)/joy", topic)
        if m is not None:
            idd = "LHR_" + m[1]
            name_list.append(idd)
    return name_list
