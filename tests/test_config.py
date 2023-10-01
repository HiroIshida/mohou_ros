import os
import shutil

import pytest
from mohou.file import get_project_path

from mohou_ros_utils.config import Config
from mohou_ros_utils.file import RelativeName, get_subpath


@pytest.fixture(scope="session")
def example_config():
    project_name = "_mohou_ros_utils_test"

    # remove project if exists
    project_path = get_project_path(project_name)
    if project_path.exists():
        shutil.rmtree(str(project_path))
    project_path.mkdir(exist_ok=True)

    # create project and  move the main_config to project dir
    get_project_path(project_name)
    here_full_filepath = os.path.join(os.getcwd(), __file__)
    here_full_dirpath = os.path.dirname(here_full_filepath)
    src = os.path.join(here_full_dirpath, "data", "main_config.yaml")
    dist = get_subpath(project_path, RelativeName.main_config)
    shutil.copyfile(src, dist)

    # move image_config to project dir
    src = os.path.join(here_full_dirpath, "data", "image_config.yaml")
    dist = get_subpath(project_path, RelativeName.image_config)
    shutil.copyfile(src, dist)

    config = Config.from_project_path(project_path)
    assert config.project_path.name == project_name
    assert len(config.topics.topic_config_list) == 7
    assert len(config.topics.use_topic_list) == 6

    assert len(config.additional_topics) == 1

    assert len(config.control_joints) == 7

    return Config.from_project_path(project_path)
