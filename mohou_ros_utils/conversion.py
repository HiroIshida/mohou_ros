from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import genpy
import numpy as np
from cv_bridge import CvBridge
from mohou.types import (
    AngleVector,
    DepthImage,
    ElementDict,
    GripperState,
    PrimitiveElementBase,
    PrimitiveElementT,
    RGBImage,
)
from mohou.utils import get_all_concrete_leaftypes

# Only pr2 user
from pr2_controllers_msgs.msg import JointControllerState
from sensor_msgs.msg import CompressedImage, Image, JointState
from tunable_filter.tunable import CompositeFilter, CropResizer, ResolutionChangeResizer

from mohou_ros_utils.config import Config
from mohou_ros_utils.utils import deprecated

MessageT = TypeVar("MessageT", bound=genpy.Message)


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

    assert encoding in ["rgb8", "bgr8"]

    # see: cv_bridge/core.py
    img_msg = Image()
    img_msg.height = data.shape[0]
    img_msg.width = data.shape[1]
    img_msg.encoding = encoding

    img_msg.data = data.tostring()  # type: ignore
    img_msg.step = len(img_msg.data) // img_msg.height

    if data.dtype.byteorder == ">":
        img_msg.is_bigendian = True
    return img_msg


@dataclass
class AbstractDataclass(ABC):
    # https://stackoverflow.com/a/60669138/7624196
    def __new__(cls, *args, **kwargs):
        if cls == AbstractDataclass or cls.__bases__[0] == AbstractDataclass:
            raise TypeError("Cannot instantiate abstract class.")
        return super().__new__(cls)


MessageConverterT = TypeVar("MessageConverterT", bound="MessageConverter")


@dataclass  # type: ignore
class MessageConverter(AbstractDataclass, Generic[PrimitiveElementT]):
    topic_name_list: List[str]

    def __post_init__(self):
        if self.__class__ == MessageConverter:
            raise TypeError("Cannot instantiate abstract class.")

    @classmethod
    def config_to_topic_name_list(cls, config: Config) -> List[str]:
        output_type = cls.out_element_type()
        topic_config = config.topics.type_config_table[output_type]
        topic_name = topic_config.name
        topic_name_list = [topic_name]  # TODO: extend to multiple message
        return topic_name_list

    @classmethod
    @abstractmethod
    def from_config(cls: Type[MessageConverterT], config: Config) -> MessageConverterT:
        pass

    @classmethod
    def from_config_topic_name_only(
        cls: Type[MessageConverterT], config: Config
    ) -> MessageConverterT:
        topic_name_list = cls.config_to_topic_name_list(config)
        return cls(topic_name_list)

    @classmethod
    def is_compatible(cls, config: Config) -> bool:
        # out elem_type
        required_output_types = list(config.topics.type_config_table.keys())
        is_out_elem_type_match = cls.out_element_type() in required_output_types
        return is_out_elem_type_match

    def is_applicable(self, msg_table: Dict[str, genpy.Message]) -> bool:
        """check if msg_table can be processed by this converter"""
        topic_name_set = set(msg_table.keys())

        # check topic_name
        required_topic_included = set(self.topic_name_list).issubset(topic_name_set)
        if not required_topic_included:
            return False

        # check topic_type
        msg_list = [msg_table[name] for name in self.topic_name_list]
        msg_type_tuple = tuple([type(msg) for msg in msg_list])
        is_type_match = tuple(self.inp_message_types()) == msg_type_tuple
        if not is_type_match:
            return False

        return True  # otherwise

    def apply_to_msg_table(
        self, msg_table: Dict[str, genpy.Message]
    ) -> Optional[PrimitiveElementT]:
        if not self.is_applicable(msg_table):
            return None

        for topic_name in self.topic_name_list:
            assert topic_name in msg_table, "{} is not given..".format(topic_name)
        # extract relevant msges
        msg_tuple = tuple([msg_table[name] for name in self.topic_name_list])
        msg_type_tuple = tuple([type(msg) for msg in msg_tuple])
        assert self.inp_message_types() == msg_type_tuple
        return self.apply(msg_tuple)  # type: ignore

    @classmethod
    @abstractmethod
    def inp_message_types(cls) -> Tuple[Type[genpy.Message]]:
        pass

    @classmethod
    @abstractmethod
    def out_element_type(cls) -> Type[PrimitiveElementT]:
        pass

    @abstractmethod
    def apply(self, msg_tuple: Tuple[genpy.Message, ...]) -> PrimitiveElementT:
        # see:
        # https://mypy.readthedocs.io/en/stable/common_issues.html#incompatible-overrides
        pass


@dataclass
class GripperStateConverter(MessageConverter[GripperState]):
    @classmethod
    def from_config(cls, config: Config):
        assert cls.is_compatible(config)
        return cls.from_config_topic_name_only(config)

    @classmethod
    def inp_message_types(cls) -> Tuple[Type[JointControllerState], ...]:  # type: ignore[override]
        return (JointControllerState,)

    @classmethod
    def out_element_type(cls) -> Type[GripperState]:
        return GripperState

    def apply(self, msg_tuple: Tuple[JointControllerState]) -> GripperState:  # type: ignore[override]
        msg = msg_tuple[0]
        return GripperState(np.array([msg.set_point]))


@dataclass
class RGBImageConverter(MessageConverter[RGBImage]):
    image_filter: Optional[CompositeFilter] = None

    @classmethod
    def from_config(cls, config: Config):
        assert cls.is_compatible(config)
        topic_name_list = cls.config_to_topic_name_list(config)
        return cls(topic_name_list, config.image_filter)

    @classmethod
    def inp_message_types(cls) -> Tuple[Type[CompressedImage]]:
        return (CompressedImage,)

    @classmethod
    def out_element_type(cls) -> Type[RGBImage]:
        return RGBImage

    def apply(self, msg_tuple: Tuple[CompressedImage]) -> RGBImage:  # type: ignore[override]
        msg = msg_tuple[0]
        image = CvBridge().compressed_imgmsg_to_cv2(msg)
        if self.image_filter is not None:
            image = self.image_filter(image)
        return RGBImage(image)


@dataclass
class DepthImageConverter(MessageConverter[DepthImage]):
    image_filter: Optional[CompositeFilter] = None

    @classmethod
    def from_config(cls, config: Config):
        assert cls.is_compatible(config)
        if config.image_filter is None:
            image_filter = None
        else:
            rgb_full_filter = config.image_filter
            rgb_full_filter.extract_subfilter([CropResizer, ResolutionChangeResizer])
            image_filter = config.image_filter
        topic_name_list = cls.config_to_topic_name_list(config)
        return cls(topic_name_list, image_filter)

    @classmethod
    def inp_message_types(cls) -> Tuple[Type[CompressedImage]]:
        return (CompressedImage,)

    @classmethod
    def out_element_type(cls) -> Type[DepthImage]:
        return DepthImage

    def apply(self, msg_tuple: Tuple[CompressedImage]) -> DepthImage:  # type: ignore[override]
        raise NotImplementedError("please make a PR.")
        """
        msg = msg_tuple[0]
        assert msg.encoding in ["32FC1"]

        size = [msg.height, msg.width]
        buf: np.ndarray = np.ndarray(
            shape=(1, int(len(msg.data) / 4)), dtype=np.float32, buffer=msg.data
        )
        image = np.nan_to_num(buf.reshape(*size))
        if self.image_filter is not None:
            assert len(self.image_filter.logical_filters) == 0
            image = self.image_filter(image, True)
        image = np.expand_dims(image, axis=2)
        return DepthImage(image)
        """


@dataclass
class AngleVectorConverter(MessageConverter[AngleVector]):
    control_joints: List[str]
    joint_indices: Optional[List[int]] = None

    @classmethod
    def from_config(cls, config: Config):
        assert cls.is_compatible(config)
        topic_name_list = cls.config_to_topic_name_list(config)
        return cls(topic_name_list, config.control_joints)

    @classmethod
    def inp_message_types(cls) -> Tuple[Type[JointState]]:
        return (JointState,)

    @classmethod
    def out_element_type(cls) -> Type[AngleVector]:
        return AngleVector

    def apply(self, msg_tuple: Tuple[JointState]) -> AngleVector:  # type: ignore[override]
        msg = msg_tuple[0]
        if self.joint_indices is None:
            name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
            self.joint_indices = [name_idx_map[name] for name in self.control_joints]

        angles = [msg.position[idx] for idx in self.joint_indices]
        return AngleVector(np.array(angles))


@dataclass
class MessageConverterCollection:
    type_to_converter_table: Dict[Type[PrimitiveElementBase], MessageConverter]

    @classmethod
    def from_config(cls, config: Config):
        all_converter_types: List[Type[MessageConverter]] = get_all_concrete_leaftypes(MessageConverter)  # type: ignore
        type_to_converter_table = {}
        for converter_type in all_converter_types:
            if converter_type.is_compatible(config):
                key = converter_type.out_element_type()
                assert key not in type_to_converter_table, "only single converter per output type"
                converter = converter_type.from_config(config)
                type_to_converter_table[key] = converter

        # check
        required_output_types = set(config.topics.type_config_table.keys())
        is_requried_output_type_match = required_output_types == set(type_to_converter_table.keys())
        assert is_requried_output_type_match
        return cls(type_to_converter_table)

    def apply_to_msg_table(self, msg_table: Dict[str, genpy.Message]) -> ElementDict:
        elem_list = []
        for converter in self.type_to_converter_table.values():
            out = converter.apply_to_msg_table(msg_table)
            if out is not None:
                elem_list.append(out)
        return ElementDict(elem_list)

    def apply(
        self, msg_tuple: Tuple[genpy.Message], output_elme_type: Type[PrimitiveElementT]
    ) -> PrimitiveElementT:
        conv = self.type_to_converter_table[output_elme_type]
        return conv.apply(msg_tuple)
