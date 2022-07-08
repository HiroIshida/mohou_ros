from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Generic, Optional, TypeVar, List, Type, Dict

from sensor_msgs.msg import JointState, CompressedImage, Image
import genpy
import numpy as np
from mohou.types import AngleVector, ElementT, ElementBase, RGBImage, DepthImage, GripperState
from tunable_filter.tunable import CompositeFilter, CropResizer, ResolutionChangeResizer
from cv_bridge import CvBridge
import cv2

from mohou_ros_utils.utils import deprecated
from mohou_ros_utils.config import Config

# Only pr2 user
from pr2_controllers_msgs.msg import JointControllerState


MessageT = TypeVar('MessageT', bound=genpy.Message)


@deprecated
def imgmsg_to_numpy(msg: Image) -> np.ndarray:  # actually numpy
    # NOTE: avoid cv_bridge for python3 on melodic
    # https://github.com/ros-perception/vision_opencv/issues/207
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    return image


@deprecated
def numpy_to_imgmsg(data: np.ndarray, encoding) -> Image:
    # NOTE: avoid cv_bridge for python3 on melodic
    # https://github.com/ros-perception/vision_opencv/issues/207

    assert encoding in ['rgb8', 'bgr8']

    # see: cv_bridge/core.py
    img_msg = Image()
    img_msg.height = data.shape[0]
    img_msg.width = data.shape[1]
    img_msg.encoding = encoding

    img_msg.data = data.tostring()  # type: ignore
    img_msg.step = len(img_msg.data) // img_msg.height

    if data.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    return img_msg


class TypeConverter(ABC, Generic[MessageT, ElementT]):
    type_in: Type[MessageT]
    type_out: Type[ElementT]

    @abstractmethod
    def __call__(self, msg: MessageT) -> ElementT:
        pass


@dataclass
class GripperStateConverter(TypeConverter[JointControllerState, GripperState]):
    type_in = JointControllerState
    type_out = GripperState

    @classmethod
    def from_config(cls, config: Config):
        return cls()

    def __call__(self, msg: JointControllerState) -> GripperState:
        return GripperState(np.array([msg.set_point]))


@dataclass
class RGBImageConverter(TypeConverter[CompressedImage, RGBImage]):
    image_filter: Optional[CompositeFilter] = None
    type_in = CompressedImage
    type_out = RGBImage

    @classmethod
    def from_config(cls, config: Config) -> 'RGBImageConverter':
        return cls(config.image_filter)

    def __call__(self, msg: CompressedImage) -> RGBImage:
        image = CvBridge().compressed_imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if self.image_filter is not None:
            image = self.image_filter(image)
        return RGBImage(image)


@dataclass
class DepthImageConverter(TypeConverter[Image, DepthImage]):
    image_filter: Optional[CompositeFilter] = None
    type_in = Image
    type_out = DepthImage

    @classmethod
    def from_config(cls, config: Config) -> 'DepthImageConverter':
        assert config.image_filter is not None
        rgb_full_filter = config.image_filter
        depth_filter = rgb_full_filter.extract_subfilter([CropResizer, ResolutionChangeResizer])
        return cls(depth_filter)

    def __call__(self, msg: Image) -> DepthImage:
        assert msg.encoding in ['32FC1']

        size = [msg.height, msg.width]
        buf: np.ndarray = np.ndarray(shape=(1, int(len(msg.data) / 4)), dtype=np.float32, buffer=msg.data)
        image = np.nan_to_num(buf.reshape(*size))
        if self.image_filter is not None:
            assert len(self.image_filter.logical_filters) == 0
            image = self.image_filter(image, True)
        image = np.expand_dims(image, axis=2)
        return DepthImage(image)


@dataclass
class AngleVectorConverter(TypeConverter[JointState, AngleVector]):
    control_joints: List[str]
    type_in = JointState
    type_out = AngleVector
    joint_indices: Optional[List[int]] = None

    @classmethod
    def from_config(cls, config: Config) -> 'AngleVectorConverter':
        return cls(config.control_joints)

    def __call__(self, msg: JointState) -> AngleVector:

        if self.joint_indices is None:
            name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
            self.joint_indices = [name_idx_map[name] for name in self.control_joints]

        angles = [msg.position[idx] for idx in self.joint_indices]
        return AngleVector(np.array(angles))


@dataclass
class VersatileConverter:
    converters: Dict[Type[ElementBase], TypeConverter]

    def __call__(self, msg: genpy.Message) -> ElementBase:

        for converter in self.converters.values():
            # image is exceptional
            if converter.type_in == type(msg):
                return converter(msg)
        assert False, 'no converter compatible with {}'.format(type(msg))

    @classmethod
    def from_config(cls, config: Config):
        converters: Dict[Type[ElementBase], TypeConverter] = {}
        converters[RGBImage] = RGBImageConverter.from_config(config)
        converters[AngleVector] = AngleVectorConverter.from_config(config)
        converters[GripperState] = GripperStateConverter.from_config(config)
        return cls(converters)
