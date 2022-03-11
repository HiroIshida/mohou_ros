import numpy as np
from rospy.rostime import Time

from mohou_ros_utils.interpolator import NearestNeighbourMessageInterpolator
from mohou_ros_utils.conversion import numpy_to_imgmsg


def test_nearest_neighbor_interpolator():
    mat = np.random.randint(0, high=255, size=(224, 224, 3)).astype(np.uint8)
    msg1 = numpy_to_imgmsg(mat, encoding='rgb8')
    msg1.header.stamp = Time.from_sec(0.0)

    mat2 = np.random.randint(0, high=255, size=(224, 224, 3)).astype(np.uint8)
    msg2 = numpy_to_imgmsg(mat2, encoding='rgb8')
    msg2.header.stamp = Time.from_sec(1.0)

    itp = NearestNeighbourMessageInterpolator.from_headered_messages([msg1, msg2])

    assert itp(Time(0.2)).data == msg1.data
    assert itp(Time(0.8)).data == msg2.data
