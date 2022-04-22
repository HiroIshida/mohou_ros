import os
import yaml
from dataclasses import dataclass
from typing import List, Dict, Optional
from tunable_filter.tunable import CompositeFilter

from mohou_ros_utils.file import get_home_position_file, get_project_dir


@dataclass
class EachTopicConfig:
    name: str
    rosbag: bool
    use: bool
    auxiliary: bool

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'EachTopicConfig':
        return cls(yaml_dict['name'], yaml_dict['rosbag'], yaml_dict['use'], yaml_dict['auxiliary'])

    def __post_init__(self):
        if self.use:
            assert self.rosbag
        if self.auxiliary:
            assert self.rosbag


@dataclass
class TopicConfig:
    rgb_topic_config: EachTopicConfig
    depth_topic_config: EachTopicConfig
    av_topic_config: EachTopicConfig

    @property
    def topic_config_list(self) -> List[EachTopicConfig]:
        return [self.rgb_topic_config, self.depth_topic_config, self.av_topic_config]

    @property
    def rosbag_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.rosbag]

    @property
    def use_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.use]

    @property
    def auxiliary_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.auxiliary]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'TopicConfig':
        return cls(
            EachTopicConfig.from_yaml_dict(yaml_dict['RGBImage']),
            EachTopicConfig.from_yaml_dict(yaml_dict['DepthImage']),
            EachTopicConfig.from_yaml_dict(yaml_dict['AngleVector']))


@dataclass
class Config:
    project: str
    control_joints: List[str]
    hz: float
    topics: TopicConfig
    home_position: Optional[Dict[str, float]]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'Config':
        project_name = yaml_dict['project']
        control_joints = yaml_dict['control_joints']
        hz = yaml_dict['sampling_hz']
        topics = TopicConfig.from_yaml_dict(yaml_dict['topic'])

        home_position = None
        home_position_file = get_home_position_file(project_name)
        if os.path.exists(home_position_file):
            with open(get_home_position_file(project_name), 'r') as f:
                home_position = yaml.safe_load(f)

        # finally load home position only if formally obtained
        return cls(
            yaml_dict['project'],
            control_joints,
            hz,
            topics,
            home_position)

    @classmethod
    def from_yaml_file(cls, file_path: str) -> 'Config':
        with open(file_path, 'r') as f:
            dic = yaml.safe_load(f)
        return cls.from_yaml_dict(dic)

    @classmethod
    def from_rospkg_path(cls, package_name: str, relative_path: str) -> 'Config':
        try:
            import rospkg
        except ImportError as e:
            e
            assert False, 'You need to intall ros. Or, maybe forget sourcing?'

        base_dir = rospkg.RosPack().get_path(package_name)
        yaml_file_path = os.path.join(base_dir, relative_path)
        return cls.from_yaml_file(yaml_file_path)

    def get_project_dir(self) -> str:
        return get_project_dir(self.project)

    def get_image_config_path(self) -> str:
        p = os.path.join(get_project_dir(self.project), 'image_config.yaml')
        return p

    def load_image_filter(self) -> CompositeFilter:
        return CompositeFilter.from_yaml(self.get_image_config_path())
