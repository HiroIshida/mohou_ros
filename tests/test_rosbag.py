import os
import tempfile

import rospy
from rosbag import Bag
from std_msgs.msg import String
from sensor_msgs.msg import Image

from mohou_ros_utils.rosbag import bag_to_synced_seqs
from mohou_ros_utils.rosbag import resolve_topic_type


def test_resolve_topic_type():
    tmp_msg_type = '_std_msgs__String'
    module_name, message_name = resolve_topic_type(tmp_msg_type)
    exec('from {} import {}'.format(module_name, message_name))


def test_sync_rosbag():

    with tempfile.TemporaryDirectory() as dname:
        filename = os.path.join(dname, 'temp.bag')
        bag = Bag(filename, mode='w')

        for i in range(12):
            time = rospy.rostime.Time.from_sec(i * 1.0)
            bag.write('dummy_string', String(), time)

            time = rospy.rostime.Time.from_sec(i * 1.2)
            bag.write('dummy_image', Image(), time)
        bag.close()

        bag = Bag(filename, mode='r')
        seqs = bag_to_synced_seqs(bag, 2.0)
        bag.close()

        assert len(seqs[0].time_list) == 6
        assert seqs[0].time_list[0] == 1.0

        bag = Bag(filename, mode='r')
        seqs = bag_to_synced_seqs(bag, 2.0, topic_names=['dummy_string'])
        bag.close()

        assert len(seqs) == 1
        assert seqs[0].object_type == String
