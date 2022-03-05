from abc import ABC, abstractmethod
import copy
from dataclasses import dataclass
from typing import List, Optional, Type, TypeVar, Generic

import numpy as np
from scipy import interpolate
import rospy
import genpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
try:
    import cv_bridge
except:
    cv_bridge = None

from mohou_ros_utils.types import TimeStampedSequence

MsgT = TypeVar('MsgT', bound=genpy.Message)


def is_sorted(list):
    # https://stackoverflow.com/questions/3755136
    return all(list[i] <= list[i+1] for i in range(len(list) - 1))


class MessageInterpolator(ABC, Generic[MsgT]):
    msg_type: Type[MsgT]
    msg_list: List[MsgT]
    time_list: List[float]

    def __init__(
            self,
            msg_list: List[MsgT],
            time_stamp_list: Optional[List[rospy.rostime.Time]] = None):

        self.msg_type = type(msg_list[0])

        if time_stamp_list is None and self.is_stamped():
            time_stamp_list = [e.header.stamp for e in msg_list]  # type: ignore
        else:
            assert time_stamp_list is not None

        time_list = [e.to_sec() for e in time_stamp_list]
        assert is_sorted(time_list), 'your time stamp is not sorted'

        self.msg_list = msg_list
        self.time_list = time_list

    @classmethod
    def from_time_stamped_sequence(cls, seq: TimeStampedSequence) -> 'MessageInterpolator':
        assert issubclass(seq.object_type, genpy.Message)
        assert not None in seq.object_list
        return cls(seq.object_list, seq.time_list)  # type: ignore

    def is_stamped(self):
        return hasattr(self.msg_type, 'header')

    def __call__(self, time_stamp: rospy.rostime.Time) -> MsgT:
        msg_itped = self._itp_impl(time_stamp)

        if hasattr(msg_itped, 'header') and isinstance(msg_itped.header, Header):
            msg_itped.header.stamp = copy.deepcopy(time_stamp)
        return msg_itped

    @abstractmethod
    def _itp_impl(self, time_stamp: rospy.rostime.Time) -> MsgT:
        pass


class NearestNeighborInterpolator(MessageInterpolator, Generic[MsgT]):

    def _itp_impl(self, time_stamp: rospy.rostime.Time) -> MsgT:
        time = time_stamp.to_sec()
        diffs = np.abs(np.array(self.time_list) - time)
        idx_closest = np.argmin(diffs)
        return copy.deepcopy(self.msg_list[idx_closest])


class VectorizationBasedInterpolator(MessageInterpolator, ABC, Generic[MsgT]):
    itp: interpolate.interp1d
    kind: str

    def __init__(self, msg_list: List[MsgT], time_stamp_list: List[rospy.rostime.Time], kind: str = 'linear'):
        assert kind in ['linear', 'cubic', 'nearest']
        super().__init__(msg_list, time_stamp_list)
        self.itp = self.crate_interpolator()

    def crate_interpolator(self) -> interpolate.interp1d:
        vector_list = [self.vectorize_msg(e) for e in self.msg_list]
        itp = interpolate.interp1d(self.time_list, vector_list, self.kind)
        return itp

    @abstractmethod
    def vectorize_msg(self, msg: MsgT) -> np.ndarray:
        pass

    @abstractmethod
    def devectorize(self, vector: np.ndarray) -> MsgT:
        pass

    def _itp_impl(self, time_stamp: rospy.rostime.Time) -> MsgT:
        itped = self.itp([time_stamp.to_sec()])[0]
        return self.devectorize(itped)


class ImageInterpolator(VectorizationBasedInterpolator, Image):
    image_tmpl: Optional[Image]

    def vectorize_msg(self, msg: Image) -> np.ndarray:
        if self.image_tmpl is None:
            self.image_tmpl = msg

        if self.image_tmpl.encoding in ['rgb8', 'bgr8']:
            bridge = cv_bridge.CvBridge()
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return img.flatten()

        assert False

    def devectorize(self, vector: np.ndarray) -> Image:
        assert self.image_tmpl is not None
        mat = vector.reshape(self.image_tmpl.height, self.image_tmpl.width)
        bridge = cv_bridge.CvBridge()
        tmp = bridge.cv2_to_imgmsg(mat, encoding=self.image_tmpl.encoding)

        msg = copy.deepcopy(self.image_tmpl)
        msg.data = tmp.data
        return msg
