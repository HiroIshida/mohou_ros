from typing import Optional

from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from mohou.types import RGBImage
from mohou.types import DepthImage
from mohou.types import ElementSequence

from mohou_ros_utils.types import TimeStampedSequence
from mohou_ros_utils.resizer import RGBResizer
from mohou_ros_utils.resizer import DepthResizer


def Image_to_RGBImage(msg: Image, resizer: RGBResizer) -> RGBImage:
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    data = resizer(image)
    return RGBImage(data)


def Image_to_DepthImage(msg: Image, resizer: DepthResizer) -> DepthImage:
    size = [msg.height, msg.width]
    buf: np.ndarray = np.ndarray(shape=(1, int(len(msg.data) / 4)), dtype=np.float32, buffer=msg.data)
    depth = buf.reshape(*size)
    data = resizer(depth)
    return DepthImage(data)


def sequence_to_element_sequence(
        seq: TimeStampedSequence,
        rgb_resizer: Optional[RGBResizer] = None,
        depth_resizer: Optional[DepthResizer] = None) -> ElementSequence:
    if seq.object_type == Image:
        if seq.object_type[0].encoding in ['bgr8', 'rgb8']:
            assert rgb_resizer is not None
            elem_seq = ElementSequence([Image_to_RGBImage(msg, rgb_resizer) for msg in seq.object_list])  # type: ignore
        elif seq.object_type[0].encoding in ['32FC1']:
            assert depth_resizer is not None
            elem_seq = ElementSequence([Image_to_DepthImage(msg, depth_resizer) for msg in seq.object_list])  # type: ignore
        raise NotImplementedError
    else:
        raise NotImplementedError
    return elem_seq
