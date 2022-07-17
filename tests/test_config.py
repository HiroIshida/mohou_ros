import os
import shutil

import pytest
from mohou.file import get_project_path

from mohou_ros_utils.config import Config
from mohou_ros_utils.file import get_image_config_path, get_main_config_path


@pytest.fixture(scope="session")
def example_config():
    project_name = "_mohou_ros_utils_test"

    # remove project if exists
    project_path = get_project_path(project_name)
    shutil.rmtree(str(project_path))

    # create project and  move the main_config to project dir
    get_project_path(project_name)
    here_full_filepath = os.path.join(os.getcwd(), __file__)
    here_full_dirpath = os.path.dirname(here_full_filepath)
    src = os.path.join(here_full_dirpath, "data", "main_config.yaml")
    dist = get_main_config_path(str(project_path))
    shutil.copyfile(src, dist)

    # move image_config to project dir
    src = os.path.join(here_full_dirpath, "data", "image_config.yaml")
    dist = get_image_config_path(str(project_path))
    shutil.copyfile(src, dist)

    return Config.from_project_name(project_name)
