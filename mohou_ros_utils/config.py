import os
import yaml
from dataclasses import dataclass
from typing import List, Dict, Optional

from mohou_ros_utils.file import get_image_config_file
from mohou_ros_utils.file import get_home_position_file


@dataclass
class EachTopicConfig:
    name: str
    rosbag: bool
    dataset: bool
    augment: bool

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'EachTopicConfig':
        return cls(yaml_dict['name'], yaml_dict['rosbag'], yaml_dict['dataset'], yaml_dict['augment'])

    def __post_init__(self):
        if self.dataset:
            assert self.rosbag
        if self.augment:
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
    def dataset_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.dataset]

    @property
    def augment_topic_list(self) -> List[str]:
        return [t.name for t in self.topic_config_list if t.augment]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'TopicConfig':
        return cls(
            EachTopicConfig.from_yaml_dict(yaml_dict['RGBImage']),
            EachTopicConfig.from_yaml_dict(yaml_dict['DepthImage']),
            EachTopicConfig.from_yaml_dict(yaml_dict['AngleVector']))


@dataclass
class ImageConfig:
    x_min: int
    x_max: int
    y_min: int
    y_max: int
    resol: int

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict, project_name: str) -> 'ImageConfig':

        image_config_file = get_image_config_file(project_name)
        if os.path.exists(image_config_file):
            with open(image_config_file, 'r') as f:
                yaml_dict_overwrite = yaml.safe_load(f)
            yaml_dict['x_min'] = yaml_dict_overwrite['x_min']
            yaml_dict['x_max'] = yaml_dict_overwrite['x_max']
            yaml_dict['y_min'] = yaml_dict_overwrite['y_min']
            yaml_dict['y_max'] = yaml_dict_overwrite['y_max']

        return cls(
            yaml_dict['x_min'],
            yaml_dict['x_max'],
            yaml_dict['y_min'],
            yaml_dict['y_max'],
            yaml_dict['resol'])


@dataclass
class Config:
    project: str
    control_joints: List[str]
    hz: float
    topics: TopicConfig
    image_config: ImageConfig
    home_position: Optional[Dict[str, float]]

    @classmethod
    def from_yaml_dict(cls, yaml_dict: Dict) -> 'Config':
        project_name = yaml_dict['project']
        control_joints = yaml_dict['control_joints']
        hz = yaml_dict['sampling_hz']
        topics = TopicConfig.from_yaml_dict(yaml_dict['topic'])
        image_config = ImageConfig.from_yaml_dict(yaml_dict['image'], project_name)

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
            image_config,
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
