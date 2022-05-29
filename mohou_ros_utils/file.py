import os


def create_if_not_exist(directory: str):
    if not os.path.exists(directory):
        os.makedirs(directory)


def get_base_dir() -> str:
    base_dir = os.path.expanduser('~/.mohou')
    create_if_not_exist(base_dir)
    return base_dir


def get_project_dir(project_name: str) -> str:
    directory = os.path.join(get_base_dir(), project_name)
    create_if_not_exist(directory)
    return directory


def get_rosbag_dir(project_name: str) -> str:
    directory = get_project_dir(project_name)
    rosbag_dir = os.path.join(directory, 'rosbag')
    create_if_not_exist(rosbag_dir)
    return rosbag_dir


def get_execution_debug_data_dir(project_name: str) -> str:
    directory = get_project_dir(project_name)
    debug_dir = os.path.join(directory, 'execution_debug_data')
    create_if_not_exist(debug_dir)
    return debug_dir


def get_home_position_file(project_name: str) -> str:
    directory = get_project_dir(project_name)
    return os.path.join(directory, 'home_position.yaml')


def get_main_config_path(project_name: str) -> str:
    directory = get_project_dir(project_name)
    return os.path.join(directory, 'main_config.yaml')


def get_image_config_path(project_name: str) -> str:
    directory = get_project_dir(project_name)
    return os.path.join(directory, 'image_config.yaml')
