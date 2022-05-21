import warnings
import functools
import numpy as np
from dataclasses import dataclass
from typing import Optional


def deprecated(func):

    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  # turn off filter
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning,
                      stacklevel=2)
        warnings.simplefilter('default', DeprecationWarning)  # reset filter
        return func(*args, **kwargs)
    return new_func


@dataclass
class CoordinateTransform:
    trans: np.ndarray
    rot: np.ndarray
    src: Optional[str] = None
    dest: Optional[str] = None

    def __call__(self, vec_src: np.ndarray) -> np.ndarray:
        return self.rot.dot(vec_src) + self.trans

    def inverse(self) -> 'CoordinateTransform':
        rot_new = self.rot.T
        trans_new = -rot_new.dot(self.trans)
        return CoordinateTransform(trans_new, rot_new, self.dest, self.src)

    @classmethod
    def from_src_coordinate_wrt_dest(cls, pos, rot) -> 'CoordinateTransform':
        return CoordinateTransform(pos, rot).inverse()


def chain_transform(tf_a2b: CoordinateTransform, tf_b2c: CoordinateTransform) -> CoordinateTransform:
    if tf_a2b.dest is not None and tf_b2c.src is not None:
        assert tf_a2b.dest == tf_b2c.src

    trans_a2c = tf_b2c.trans + tf_b2c.rot.dot(tf_a2b.trans)
    rot_a2c = tf_b2c.rot.dot(tf_a2b.rot)

    src_new = tf_a2b.src
    dest_new = tf_b2c.dest
    return CoordinateTransform(trans_a2c, rot_a2c, src_new, dest_new)
