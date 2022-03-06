import os
import pickle

from mohou_ros_utils.resizer import RGBResizer
from mohou_ros_utils.resizer import DepthResizer
from mohou_ros_utils.type_conversion import RGBImageConverter
from mohou_ros_utils.type_conversion import DepthImageConverter
from mohou_ros_utils.type_conversion import AngleVectorConverter


def get_test_data_path():
    here_full_filepath = os.path.join(os.getcwd(), __file__)
    here_full_dirpath = os.path.dirname(here_full_filepath)
    return os.path.join(here_full_dirpath, 'data')


def get_pickle_data_path(pickle_name):
    path = os.path.join(get_test_data_path(), pickle_name)
    if not os.path.exists(path):
        cmd = 'bash ' + os.path.join(get_test_data_path(), 'download_data.sh')
        os.system(cmd)

    with open(path, 'rb') as f:
        obj = pickle.load(f)
    return obj


def test_rgb_image_converter():
    rgb_image_msg = get_pickle_data_path('rgb_image.pkl')
    args = [10, 300, 10, 300, 24]
    rgb_converter = RGBImageConverter(RGBResizer(*args))
    image = rgb_converter(rgb_image_msg)
    assert image.shape == (24, 24, 3)


def test_depth_image_converter():
    with open(os.path.join(get_test_data_path(), 'depth_image.pkl'), 'rb') as f:
        depth_image_msg = pickle.load(f)

    args = [10, 100, 10, 100, 24]
    depth_converter = DepthImageConverter(DepthResizer(*args))
    image = depth_converter(depth_image_msg)
    assert image.shape == (24, 24, 1)


def test_angle_vector_converter():
    # joint state of pr2
    with open(os.path.join(get_test_data_path(), 'joint_states.pkl'), 'rb') as f:
        joint_state = pickle.load(f)

    control_joints = ['r_wrist_flex_joint', 'r_wrist_roll_joint']
    converter = AngleVectorConverter(control_joints)

    av = converter(joint_state)
    assert av.shape == (2,)