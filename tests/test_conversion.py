import os
import pickle

import numpy as np
import pytest
from mohou.types import AngleVector, GripperState, RGBImage
from pr2_controllers_msgs.msg import JointControllerState
from sensor_msgs.msg import CompressedImage, JointState
from test_config import example_config  # noqa

from mohou_ros_utils.conversion import (
    AngleVectorConverter,
    AnotherGripperState,
    DepthImageConverter,
    GripperStateConverter,
    MessageConverterCollection,
    RGBImageConverter,
    imgmsg_to_numpy,
    numpy_to_imgmsg,
)


def test_imgmsg_cv2_conversion():
    mat = np.random.randint(0, high=255, size=(224, 224, 3)).astype(np.uint8)
    msg = numpy_to_imgmsg(mat, "rgb8")
    mat_again = imgmsg_to_numpy(msg)
    np.testing.assert_almost_equal(mat, mat_again)

    msg_again = numpy_to_imgmsg(mat_again, "rgb8")
    assert msg.height == msg_again.height
    assert msg.width == msg_again.width
    assert msg.step == msg_again.step
    assert msg.encoding == msg_again.encoding
    assert msg.is_bigendian == msg_again.is_bigendian
    assert msg.data == msg_again.data


def get_test_data_path():
    here_full_filepath = os.path.join(os.getcwd(), __file__)
    here_full_dirpath = os.path.dirname(here_full_filepath)
    return os.path.join(here_full_dirpath, "data")


def get_pickle_data_path(pickle_name):
    path = os.path.join(get_test_data_path(), pickle_name)
    if not os.path.exists(path):
        cmd = "bash " + os.path.join(get_test_data_path(), "download_data.sh")
        os.system(cmd)

    with open(path, "rb") as f:
        obj = pickle.load(f)
    return obj


def test_rgb_image_converter(example_config):  # type: ignore # noqa
    config = example_config
    rgb_image_msg: CompressedImage = get_pickle_data_path("rgb_image.pkl")
    rgb_converter = RGBImageConverter.from_config(config)

    # ok case
    msg_table = {"/kinect_head/rgb/image_rect_color": rgb_image_msg}
    image = rgb_converter.apply_to_msg_table(msg_table)  # type: ignore
    assert image is not None
    assert image.shape == (112, 112, 3)

    # incompatible case, topic_name
    msg_table = {"/hoge_kinect_head/rgb/image_rect_color": rgb_image_msg}
    image = rgb_converter.apply_to_msg_table(msg_table)  # type: ignore
    assert image is None

    # incompatible case, topic_type
    incompat_msg = JointState()
    msg_table = {"/kinect_head/rgb/image_rect_color": incompat_msg}
    with pytest.raises(AssertionError):
        image = rgb_converter.apply_to_msg_table(msg_table)  # type: ignore


def test_depth_image_converter(example_config):  # type: ignore # noqa
    config = example_config
    get_pickle_data_path("depth_image.pkl")
    DepthImageConverter.from_config(config)
    # TODO: add test when you implement DepthImageConverter


def test_angle_vector_converter(example_config):  # type: ignore # noqa
    # joint state of pr2
    config = example_config
    joint_state: JointState = get_pickle_data_path("joint_states.pkl")

    msg_table = {"/joint_states": joint_state}
    converter = AngleVectorConverter.from_config(config)
    av = converter.apply_to_msg_table(msg_table)  # type: ignore
    assert av is not None
    assert av.shape == (7,)


def test_gripper_state_converter(example_config):  # type: ignore # noqa
    config = example_config
    gripper_state_msg = JointControllerState()
    gripper_state_msg.set_point = 1.0  # whatever
    msg_table = {"/r_gripper_controller/state": gripper_state_msg}
    converter = GripperStateConverter.from_config(config)
    out = converter.apply_to_msg_table(msg_table)  # type: ignore
    assert out is not None
    np.testing.assert_almost_equal(out.numpy(), np.array([1.0]))


def test_overall_message_converter(example_config):  # type: ignore # noqa
    config = example_config
    rgb_image_msg: CompressedImage = get_pickle_data_path("rgb_image.pkl")
    joint_state_msg: JointState = get_pickle_data_path("joint_states.pkl")
    gripper_state_msg = JointControllerState(set_point=1.0)
    gripper_state_msg2 = JointControllerState(set_point=1.0)
    msg_table = {
        "/kinect_head/rgb/image_rect_color": rgb_image_msg,
        "/joint_states": joint_state_msg,
        "/r_gripper_controller/state": gripper_state_msg,
        "/l_gripper_controller/state": gripper_state_msg2,
        "/hogehoge": gripper_state_msg,  # this will just ignored
    }
    conv = MessageConverterCollection.from_config(config)
    out = conv.apply_to_msg_table(msg_table)

    assert set(out.keys()) == {RGBImage, AngleVector, GripperState, AnotherGripperState}
    assert out[RGBImage].shape == (112, 112, 3)
    assert out[AngleVector].shape == (7,)
    assert out[GripperState].shape == (1,)
    assert out[AnotherGripperState].shape == (1,)
