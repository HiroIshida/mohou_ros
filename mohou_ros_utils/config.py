from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Type

import yaml
from mohou.types import PrimitiveElementBase, get_element_type
from tunable_filter.tunable import CompositeFilter

from mohou_ros_utils.file import RelativeName, get_subpath


@dataclass
class EachTopicConfig:
    mohou_type: Type[PrimitiveElementBase]
    topic_name_list: List[str]
    rosbag: bool
    use: bool

    @classmethod
    def from_yaml_dict(
        cls, yaml_dict: Dict, mohou_type: Type[PrimitiveElementBase]
    ) -> "EachTopicConfig":
        partial_yaml_dict = yaml_dict[mohou_type.__name__]
        name_list = partial_yaml_dict["name"]
        if not isinstance(name_list, list):
            name_list = [name_list]
        return cls(
            mohou_type,
            name_list,
            partial_yaml_dict["rosbag"],
            partial_yaml_dict["use"],
        )

    def __post_init__(self):
        if self.use:
            assert self.rosbag


@dataclass
class TopicConfig:
    type_config_table: Dict[Type[PrimitiveElementBase], EachTopicConfig]

    @property
    def topic_config_list(self) -> List[EachTopicConfig]:
        return list(self.type_config_table.values())

    @property
    def rosbag_topic_list(self) -> List[str]:
        topic_list = []
        for topic_config in self.topic_config_list:
            if topic_config.rosbag:
                topic_list.extend(topic_config.topic_name_list)
        return topic_list

    @property
    def use_topic_list(self) -> List[str]:
        topic_list = []
        for topic_config in self.topic_config_list:
            if topic_config.use:
                topic_list.extend(topic_config.topic_name_list)
        return topic_list

    def get_by_mohou_type(self, mohou_type: Type[PrimitiveElementBase]) -> EachTopicConfig:
        return self.type_config_table[mohou_type]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> "TopicConfig":
        type_config_table: Dict[Type[PrimitiveElementBase], EachTopicConfig] = {}
        for key in yaml_dict.keys():
            type_key: Type[PrimitiveElementBase] = get_element_type(key)  # type: ignore
            type_config_table[type_key] = EachTopicConfig.from_yaml_dict(yaml_dict, type_key)
        return cls(type_config_table)


@dataclass
class Config:
    project_path: Path
    control_joints: List[str]
    topics: TopicConfig
    home_position: Optional[Dict[str, float]]
    image_filter: Optional[CompositeFilter]

    @classmethod
    def from_project_path(cls, project_path: Path) -> "Config":
        main_config_path = get_subpath(project_path, RelativeName.main_config)
        with main_config_path.open(mode="r") as f:
            main_config_dict = yaml.safe_load(f)
        control_joints = main_config_dict["control_joints"]
        topics = TopicConfig.from_yaml_dict(main_config_dict["topic"])

        # maybe not set
        home_position_path = get_subpath(project_path, RelativeName.home_position)
        if home_position_path.exists():
            with home_position_path.open(mode="r") as f:
                home_position: Optional[Dict] = yaml.safe_load(f)
        else:
            home_position = None

        # maybe not set
        image_config_path = get_subpath(project_path, RelativeName.image_config)
        if image_config_path.exists():
            image_filter: Optional[CompositeFilter] = CompositeFilter.from_yaml(
                str(image_config_path)
            )
        else:
            image_filter = None

        return cls(project_path, control_joints, topics, home_position, image_filter)
