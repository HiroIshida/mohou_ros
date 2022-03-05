from abc import ABC, abstractmethod
import numpy as np
import scipy.interpolate
import cv2


class AbstractResizer(ABC):
    x_bound: slice
    y_bound: slice
    resol: int

    def __init__(self, x_min, x_max, y_min, y_max, resol):
        self.x_bound = slice(x_min, x_max)
        self.y_bound = slice(y_min, y_max)
        self.resol = resol

    @abstractmethod
    def __call__(self, image: np.ndarray) -> np.ndarray:
        pass


class RGBResizer(AbstractResizer):

    def __call__(self, image: np.ndarray) -> np.ndarray:
        image_resized = image[self.x_bound, self.y_bound]
        image_final = cv2.resize(image_resized, (self.resol, self.resol), interpolation=cv2.INTER_AREA)
        return image_final


class DepthResizer(AbstractResizer):

    def __call__(self, image) -> np.ndarray:

        image_resized = image[self.x_bound, self.y_bound]

        def create_mesh_points(width, height):
            xlin = np.linspace(0, 1., width)
            ylin = np.linspace(0, 1., height)
            xx, yy = np.meshgrid(xlin, ylin)
            pts = np.array(list(zip(xx.flatten(), yy.flatten())))
            return pts

        pts_in = create_mesh_points(*image_resized.shape)
        pts_out = create_mesh_points(self.resol, self.resol)

        itped = scipy.interpolate.griddata(pts_in, image_resized.T.flatten(), pts_out)
        itped_reshaped = itped.reshape(self.resol, self.resol).T
        return itped_reshaped
