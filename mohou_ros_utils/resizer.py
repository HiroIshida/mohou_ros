from abc import ABC, abstractmethod
import numpy as np
import cv2

from mohou_ros_utils.config import ImageConfig


class AbstractResizer(ABC):
    x_bound: slice
    y_bound: slice
    resol: int

    def __init__(self, x_min, x_max, y_min, y_max, resol):
        self.x_bound = slice(x_min, x_max)
        self.y_bound = slice(y_min, y_max)
        self.resol = resol

    @classmethod
    def from_config(cls, config: ImageConfig):
        return cls(config.x_min, config.x_max, config.y_min, config.y_max, config.resol)

    @abstractmethod
    def __call__(self, image: np.ndarray) -> np.ndarray:
        pass


class RGBResizer(AbstractResizer):

    def __call__(self, image: np.ndarray) -> np.ndarray:
        image_resized = image[self.x_bound, self.y_bound]
        image_final = cv2.resize(image_resized, (self.resol, self.resol), interpolation=cv2.INTER_AREA)
        return image_final


class DepthResizer(AbstractResizer):

    def __call__(self, image, method=cv2.INTER_CUBIC) -> np.ndarray:

        image_resized = image[self.x_bound, self.y_bound]
        image_final = cv2.resize(image_resized, (self.resol, self.resol), interpolation=method)
        return image_final
