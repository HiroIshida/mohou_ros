#!/usr/bin/env python3
import pickle

import rospy
from sensor_msgs.msg import CompressedImage, Image, JointState

data_config = [
    ("/kinect_head/depth_registered/image", Image, "depth_image.pkl"),
    ("/kinect_head/rgb/image_rect_color/compressed", CompressedImage, "rgb_image.pkl"),
    ("/joint_states", JointState, "joint_states.pkl"),
]


if __name__ == "__main__":
    rospy.init_node("tmp")

    def create_callback(topic_name, pkl_name, flags):
        def callback(msg):
            if flags[topic_name]:
                return
            print("saving {}".format(topic_name))
            with open(pkl_name, "wb") as f:
                pickle.dump(msg, f)
            flags[topic_name] = True

        return callback

    flags = {}
    for topic_name, topic_type, pkl_name in data_config:
        flags[topic_name] = False
        cb = create_callback(topic_name, pkl_name, flags)
        rospy.Subscriber(topic_name, topic_type, callback=cb)

    rospy.sleep(4.0)
