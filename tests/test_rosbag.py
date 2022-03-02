import os
import tempfile

import rospy
from rosbag import Bag
from std_msgs.msg import String
from sensor_msgs.msg import Image

from mohou_ros_utils.rosbag import bag_to_seqs, bag_to_synced_seqs

def test_sync_rosbag():

    with tempfile.TemporaryDirectory() as dname:
        filename = os.path.join(dname, 'temp.bag') 
        bag = Bag(filename, mode='w')

        for i in range(100):
            time = rospy.rostime.Time.from_sec(i * 1.0)
            bag.write('dummy_string', String(), time)

            time = rospy.rostime.Time.from_sec(i * 1.2)
            bag.write('dummy_image', Image(), time)
        bag.close()

        bag = Bag(filename, mode='r')
        seqs = bag_to_synced_seqs(bag, 2.0)
