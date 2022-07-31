from enum import Enum
from pathlib import Path


class RelativeName(Enum):
    rosbag = ("rosbag", True)
    exec_debug = ("execution_debug_data", True)
    home_position = ("home_position.yaml", False)
    main_config = ("main_config.yaml", False)
    image_config = ("image_config.yaml", False)


def get_subpath(project_path: Path, relative_name: RelativeName) -> Path:
    ret = relative_name.value
    name: str = ret[0]  # type: ignore
    is_directory: bool = ret[1]  # type: ignore
    full_path: Path = project_path / name
    if is_directory:
        full_path.mkdir(exist_ok=True)
    return full_path
