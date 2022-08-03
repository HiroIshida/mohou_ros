#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
import cv_bridge
import numpy as np
import rospkg
import rospy
from mohou.types import RGBImage
from sensor_msgs.msg import CompressedImage

if __name__ == "__main__":
    pack_path = Path(rospkg.RosPack().get_path("mohou_ros"))
    image_path = pack_path / "rostest" / "data" / "sample.png"
    assert image_path.exists()
    cv_image = cv2.imread(str(image_path))
    image_base = RGBImage(cv_image)

    rospy.init_node("random_image_publisher", anonymous=True)
    pub = rospy.Publisher("/sample_image/compressed", CompressedImage, queue_size=10)

    while not rospy.is_shutdown():
        bridge = cv_bridge.CvBridge()
        image_rand = image_base.randomize()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode(".jpg", image_rand.numpy())[1]).tobytes()
        pub.publish(msg)
        time.sleep(0.03)  # TODO: use rospy.Rate
