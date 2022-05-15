import os
import yaml
from dataclasses import dataclass
from typing import List, Dict, Optional, Type
from tunable_filter.tunable import CompositeFilter

from mohou.types import get_element_type
from mohou.types import PrimitiveElementBase
from mohou_ros_utils.file import get_home_position_file, get_main_config_path, get_image_config_path


@dataclass
class EachTopicConfig:
    mohou_type: Type[PrimitiveElementBase]
    name: str
    rosbag: bool
    use: bool
    auxiliary: bool

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict, mohou_type: Type[PrimitiveElementBase]) -> 'EachTopicConfig':
        partial_yaml_dict = yaml_dict[mohou_type.__name__]
        return cls(
            mohou_type,
            partial_yaml_dict['name'],
            partial_yaml_dict['rosbag'],
            partial_yaml_dict['use'],
            partial_yaml_dict['auxiliary'])

    def __post_init__(self):
        if self.use:
            assert self.rosbag
        if self.auxiliary:
            assert self.rosbag


@dataclass
class TopicConfig:
    type_config_table: Dict[Type[PrimitiveElementBase], EachTopicConfig]
    name_config_table: Dict[str, EachTopicConfig]

    @property
    def topic_config_list(self) -> List[EachTopicConfig]:
        return list(self.type_config_table.values())

    @property
    def rosbag_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.rosbag]

    @property
    def use_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.use]

    @property
    def auxiliary_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.auxiliary]

    def get_by_mohou_type(self, mohou_type: Type[PrimitiveElementBase]) -> EachTopicConfig:
        return self.type_config_table[mohou_type]

    def get_by_topic_name(self, name: str) -> EachTopicConfig:
        return self.name_config_table[name]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'TopicConfig':
        type_config_table: Dict[Type[PrimitiveElementBase], EachTopicConfig] = {}
        for key in yaml_dict.keys():
            type_key: Type[PrimitiveElementBase] = get_element_type(key)  # type: ignore
            type_config_table[type_key] = EachTopicConfig.from_yaml_dict(yaml_dict, type_key)

        # create name config table
        name_config_table: Dict[str, EachTopicConfig] = {}
        for key, val in type_config_table.items():
            name_config_table[val.name] = val
        return cls(type_config_table, name_config_table)


@dataclass
class Config:
    control_joints: List[str]
    topics: TopicConfig
    home_position: Optional[Dict[str, float]]
    image_filter: Optional[CompositeFilter]

    @classmethod
    def from_project_name(cls, project_name: str) -> 'Config':
        main_config_path = get_main_config_path(project_name)
        with open(main_config_path, 'r') as f:
            main_config_dict = yaml.safe_load(f)
        control_joints = main_config_dict['control_joints']
        topics = TopicConfig.from_yaml_dict(main_config_dict['topic'])

        # maybe not set
        home_position = None
        home_position_file = get_home_position_file(project_name)
        if os.path.exists(home_position_file):
            with open(home_position_file, 'r') as f:
                home_position = yaml.safe_load(f)

        # maybe not set
        image_filter = None
        image_config_path = get_image_config_path(project_name)
        if os.path.exists(image_config_path):
            image_filter = CompositeFilter.from_yaml(image_config_path)

        return cls(control_joints, topics, home_position, image_filter)

    def load_image_filter(self) -> CompositeFilter:
        return self.image_filter
