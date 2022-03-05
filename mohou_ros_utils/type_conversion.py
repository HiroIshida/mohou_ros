from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from sensor_msgs.msg import Image
import genpy
import numpy as np
from cv_bridge import CvBridge
from mohou.types import ElementT
from mohou.types import RGBImage
from mohou.types import DepthImage

from mohou_ros_utils.resizer import RGBResizer
from mohou_ros_utils.resizer import DepthResizer


MessageT = TypeVar('MessageT', bound=genpy.Message)


class TypeConverter(ABC, Generic[MessageT, ElementT]):

    @abstractmethod
    def __call__(self, msg: MessageT) -> ElementT:
        pass


class RGBImageConverter(TypeConverter[Image, RGBImage]):
    resizer: RGBResizer

    def __init__(self, resizer: RGBResizer):
        self.resizer = resizer

    def __call__(self, msg: Image) -> RGBImage:
        assert msg.encoding in ['bgr8', 'rgb8']

        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        data = self.resizer(image)
        return RGBImage(data)


class DepthImageConverter(TypeConverter[Image, DepthImage]):
    resizer: DepthResizer

    def __init__(self, resizer: DepthResizer):
        self.resizer = resizer

    def __call__(self, msg: Image) -> DepthImage:
        assert msg.encoding in ['32FC1']

        size = [msg.height, msg.width]
        buf: np.ndarray = np.ndarray(shape=(1, int(len(msg.data) / 4)), dtype=np.float32, buffer=msg.data)
        depth = np.nan_to_num(buf.reshape(*size))
        data = np.expand_dims(self.resizer(depth), axis=2)
        return DepthImage(data)
