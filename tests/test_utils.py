import math

import numpy as np
from skrobot.coordinates.math import rotation_matrix_from_rpy
from skrobot.models import PR2

from mohou_ros_utils.utils import (
    CoordinateTransform,
    chain_transform,
    euslisp_unit_to_standard_unit,
    standard_unit_to_euslisp_unit,
)


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


def test_transform_coversion():
    mat = rotation_matrix_from_rpy([math.pi * 0.25, 0.0, 0.0])
    tf = CoordinateTransform(np.array([1, 0, 0]), mat, "b", "a")
    tf_again = CoordinateTransform.from_skrobot_coords(tf.to_skrobot_coords())
    np.testing.assert_almost_equal(tf.trans, tf_again.trans)

    tf_again = CoordinateTransform.from_ros_pose(tf.to_ros_pose())
    np.testing.assert_almost_equal(tf.trans, tf_again.trans)


def test_unit_conversion():
    # standard_unit_to_euslisp_unit

    robot_model = PR2()
    robot_model.reset_manip_pose()
    joint_name_list = [
        "torso_lift_joint",
        "l_shoulder_pan_joint",
        "l_shoulder_lift_joint",
        "l_upper_arm_roll_joint",
        "l_elbow_flex_joint",
        "l_forearm_roll_joint",
        "l_wrist_flex_joint",
        "l_wrist_roll_joint",
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
        "head_pan_joint",
        "head_tilt_joint",
    ]

    angles = np.array([robot_model.__dict__[name].joint_angle() for name in joint_name_list])
    euslisp_angles = standard_unit_to_euslisp_unit(robot_model, joint_name_list, angles)
    assert len(angles) == len(euslisp_angles)
    euslisp_angles_ref = np.array(
        [
            300.0,
            75.0,
            50.0,
            110.0,
            -110.0,
            -20.0,
            -10.0,
            -10.0,
            -75.0,
            50.0,
            -110.0,
            -110.0,
            20.0,
            -10.0,
            -10.0,
            0.0,
            50.0,
        ]
    )
    np.testing.assert_almost_equal(euslisp_angles, euslisp_angles_ref)

    standard_angles = euslisp_unit_to_standard_unit(
        robot_model, joint_name_list, euslisp_angles_ref
    )
    np.testing.assert_almost_equal(standard_angles, angles)
