import math

import numpy as np
from skrobot.coordinates.math import rotation_matrix_from_rpy

from mohou_ros_utils.utils import CoordinateTransform, chain_transform


def test_transform():
    mat = rotation_matrix_from_rpy([math.pi * 0.25, 0.0, 0.0])
    tf_b2a = CoordinateTransform(np.array([1, 0, 0]), mat, "b", "a")
    tf_c2b = CoordinateTransform(np.array([1, 0, 0]), mat, "c", "b")
    tf_c2a = chain_transform(tf_c2b, tf_b2a)

    trans = np.array([1.0 + 0.5 * np.sqrt(2.0), 0.5 * np.sqrt(2.0), 0.0])
    np.testing.assert_almost_equal(tf_c2a.trans, trans, decimal=8)
    np.testing.assert_almost_equal(tf_c2a.rot, mat.dot(mat), decimal=8)

    tf_a2c = tf_c2a.inverse()
    tf_identical = chain_transform(tf_a2c, tf_c2a)
    np.testing.assert_almost_equal(tf_identical.trans, np.zeros(3), decimal=8)
    np.testing.assert_almost_equal(tf_identical.rot, np.eye(3), decimal=8)
