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
