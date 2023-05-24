from __future__ import annotations
import math
from typing import Optional, TYPE_CHECKING

from matplotlib.patches import Circle, Ellipse, Rectangle
from aux.global_config import GlobalConfig
from components.point import Point

if TYPE_CHECKING:
    from components.object import Object


class Shape:
    def __init__(
        self,
        shape_type: str,
        significant_length: float,
        center: Point,
        angle: Optional[float] = None,
        secondary_length: Optional[float] = None,
    ):
        self.shape_type = shape_type
        self.significant_length = (
            1 + GlobalConfig.BOUNDARY_SAFETY_COEFF
        ) * significant_length
        self.angle = angle
        self.center = center
        self.secondary_length = secondary_length

    def get_patch(self, segment: Object, color: str = "r", alpha: float = 0.3):
        if self.shape_type == "circle":
            return Circle(
                (self.center.x, self.center.y),
                self.significant_length / 2,
                color=color,
                alpha=alpha,
            )
        if self.shape_type == "rectangle":
            return Rectangle(
                xy=(self.center.x, self.center.y),
                width=self.significant_length,
                height=self.significant_length,
                angle=180 * (segment.center.angle + self.angle) / math.pi,  # type: ignore
                color=color,
                alpha=alpha,
            )
        if self.shape_type == "ellipse":
            if self.secondary_length is None:
                raise Exception("missing secondary length")
            return Ellipse(
                xy=(self.center.x, self.center.y),
                width=self.significant_length,
                height=self.secondary_length,
                angle=self.angle,  # type: ignore
                color=color,
                alpha=alpha,
            )