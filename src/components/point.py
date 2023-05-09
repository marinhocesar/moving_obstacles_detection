import math
from typing import Tuple


class Point:
    def __init__(self, range: float, angle: float):
        self.range = range
        self.angle = angle
        # self.angle = angle if angle >= 0 else (2 * math.pi + angle)
        self.x, self.y = self.get_rectangular()

    def get_rectangular(self) -> Tuple[float, float]:
        x = self.range * math.cos(self.angle)
        y = self.range * math.sin(self.angle)
        return x, y

    def normalize(self) -> "Point":
        mag = self.range
        if mag == 0:
            return self

        return Point.init_from_rectangular(x=self.x / mag, y=self.y / mag)

    def scale(self, factor: float) -> "Point":
        if self.range == 0:
            return self
        return Point.init_from_rectangular(x=self.x * abs(factor), y=self.y * abs(factor))

    @classmethod
    def init_from_rectangular(cls, x: float, y: float):
        radius = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)
        return cls(range=radius, angle=angle)

    def __eq__(self, __value: "Point") -> bool:
        return self.range == __value.range and self.angle == __value.angle

    def __hash__(self) -> int:
        return hash(("range", self.range, "angle", self.angle))

    def __repr__(self) -> str:
        x, y = self.get_rectangular()
        return f"(\n\trange:{self.range},\n\tangle:{self.angle},\n\tx:{x},\n\ty:{y}\n)"
