from abc import ABC, abstractmethod
from typing import Generic, Optional, TypeVar, List
import math

from sensor_msgs.msg import Image, JointState
import genpy
import numpy as np
from mohou.types import AngleVector, ElementT, RGBImage, DepthImage

from mohou_ros_utils.resizer import RGBResizer
from mohou_ros_utils.resizer import DepthResizer


MessageT = TypeVar('MessageT', bound=genpy.Message)


def imgmsg_to_numpy(msg: Image) -> np.ndarray:  # actually numpy
    # NOTE: avoid cv_bridge for python3 on melodic
    # https://github.com/ros-perception/vision_opencv/issues/207
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    return image


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

    @abstractmethod
    def __call__(self, msg: MessageT) -> ElementT:
        pass


class RGBImageConverter(TypeConverter[Image, RGBImage]):
    resizer: Optional[RGBResizer]

    def __init__(self, resizer: Optional[RGBResizer] = None):
        self.resizer = resizer

    def __call__(self, msg: Image) -> RGBImage:
        assert msg.encoding in ['bgr8', 'rgb8']
        image = imgmsg_to_numpy(msg)
        if self.resizer is not None:
            image = self.resizer(image)
        return RGBImage(image)


class DepthImageConverter(TypeConverter[Image, DepthImage]):
    resizer: Optional[DepthResizer]

    def __init__(self, resizer: Optional[DepthResizer] = None):
        self.resizer = resizer

    def __call__(self, msg: Image) -> DepthImage:
        assert msg.encoding in ['32FC1']

        size = [msg.height, msg.width]
        buf: np.ndarray = np.ndarray(shape=(1, int(len(msg.data) / 4)), dtype=np.float32, buffer=msg.data)
        image = np.nan_to_num(buf.reshape(*size))
        if self.resizer is not None:
            image = self.resizer(image)
        image = np.expand_dims(image, axis=2)
        return DepthImage(image)


class AngleVectorConverter(TypeConverter[JointState, AngleVector]):
    control_joints: List[str]
    joint_indices: Optional[List[int]] = None

    def __init__(self, control_joints: List[str]):
        self.control_joints = control_joints

    def __call__(self, msg: JointState) -> AngleVector:

        if self.joint_indices is None:
            name_idx_map = {name: i for (i, name) in enumerate(msg.name)}
            self.joint_indices = [name_idx_map[name] for name in self.control_joints]

        def clamp_to_s1(something):
            lower_side = -math.pi
            return ((something - lower_side) % (2 * math.pi)) + lower_side

        angles = [clamp_to_s1(msg.position[idx]) for idx in self.joint_indices]
        return AngleVector(np.array(angles))
