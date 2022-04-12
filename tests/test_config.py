import os
from mohou_ros_utils.config import Config


def get_test_data_path():
    here_full_filepath = os.path.join(os.getcwd(), __file__)
    here_full_dirpath = os.path.dirname(here_full_filepath)
    return os.path.join(here_full_dirpath, 'data')


def test_load_config():
    example_yaml_file_path = os.path.join(get_test_data_path(), 'example.yaml')
    Config.from_yaml_file(example_yaml_file_path)
