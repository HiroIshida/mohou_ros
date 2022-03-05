from abc import ABC, abstractmethod
import copy
from typing import List, Optional, Type, TypeVar, Generic

import numpy as np
from scipy import interpolate
import rospy
import genpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
try:
    import cv_bridge
except ImportError:
    cv_bridge = None

from mohou_ros_utils.types import TimeStampedSequence

ObjectT = TypeVar('ObjectT')
MessageT = TypeVar('MessageT', bound=genpy.Message)


def is_sorted(list):
    # https://stackoverflow.com/questions/3755136
    return all(list[i] <= list[i + 1] for i in range(len(list) - 1))


class AbstractInterpolator(ABC, Generic[ObjectT]):
    object_type: Type[ObjectT]
    object_list: List[ObjectT]
    time_list: List[float]

    def __init__(
            self,
            object_list: List[ObjectT],
            time_list: List[float]):

        self.object_type = type(object_list[0])
        assert is_sorted(time_list), 'your time stamp is not sorted'

        self.object_list = object_list
        self.time_list = time_list

    @classmethod
    def from_time_stamped_sequence(cls, seq: TimeStampedSequence) -> 'AbstractInterpolator':
        indices_valid = [idx for idx in range(len(seq)) if seq.object_list[idx] is not None]

        time_list_valid = [seq.time_list[i] for i in indices_valid]
        object_list_valid = [seq.object_list[i] for i in indices_valid]
        return cls(object_list_valid, time_list_valid)  # type: ignore

    @abstractmethod
    def apply(self, t: float) -> ObjectT:
        pass


class ROSMessageMixin(AbstractInterpolator[MessageT]):

    @classmethod
    def from_headered_messages(cls, messages: List[MessageT]) -> 'ROSMessageMixin':
        assert hasattr(messages[0], 'header')
        time_list = [msg.header.stamp.to_sec() for msg in messages]
        return cls(messages, time_list)

    def __call__(self, time_stamp: rospy.rostime.Time) -> MessageT:
        msg_itped = self.apply(time_stamp.to_sec())
        if hasattr(msg_itped, 'header') and isinstance(msg_itped.header, Header):
            msg_itped.header.stamp = copy.deepcopy(time_stamp)
        return msg_itped


class NearestNeighbourInterpolator(AbstractInterpolator, Generic[ObjectT]):

    def apply(self, t: float) -> ObjectT:
        diffs = np.abs(np.array(self.time_list) - t)
        idx_closest = np.argmin(diffs)
        return copy.deepcopy(self.object_list[idx_closest])


class NearestNeighbourMessageInterpolator(NearestNeighbourInterpolator, ROSMessageMixin):
    pass


class VectorizationBasedInterpolator(AbstractInterpolator, ABC, Generic[ObjectT]):
    itp: interpolate.interp1d
    kind: str

    def __init__(self, msg_list: List[ObjectT], time_stamp_list: List[rospy.rostime.Time], kind: str = 'linear'):
        assert kind in ['linear', 'cubic', 'nearest']
        super().__init__(msg_list, time_stamp_list)
        self.itp = self.crate_interpolator()

    def crate_interpolator(self) -> interpolate.interp1d:
        vector_list = [self.vectorize_msg(e) for e in self.object_list]
        itp = interpolate.interp1d(self.time_list, vector_list, self.kind)
        return itp

    @abstractmethod
    def vectorize_msg(self, msg: ObjectT) -> np.ndarray:
        pass

    @abstractmethod
    def devectorize(self, vector: np.ndarray) -> ObjectT:
        pass

    def _itp_impl(self, time_stamp: rospy.rostime.Time) -> ObjectT:
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


class AbstractInterpolationRule:

    @abstractmethod
    def apply(self, seq: TimeStampedSequence) -> None:
        pass


class NullInterpolationRule(AbstractInterpolationRule):

    def apply(self, seq: TimeStampedSequence) -> None:
        pass


class AllSameInterpolationRule(AbstractInterpolationRule):
    """Apply the same interpolator independent on message types"""
    itp_type: Type[AbstractInterpolator]

    def __init__(self, itp_type: Type[AbstractInterpolator]):
        self.itp_type = itp_type

    def apply(self, seq: TimeStampedSequence) -> None:
        """fill all None object by specified interpolation method"""
        itp = self.itp_type.from_time_stamped_sequence(seq)
        for i in range(len(seq)):
            if seq.object_list[i] is None:
                seq.object_list[i] = itp.apply(seq.time_list[i])
