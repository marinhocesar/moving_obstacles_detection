from decimal import Decimal
import math
from typing import List, Optional
import numpy as np

from components.object import Object
from components.point import Point


def linear_function(x, a, b):
    y = a * x + b
    return y


def get_distance_between_data_points(first_data_point: Point, second_data_point: Point):
    return math.sqrt(
        (second_data_point.x - first_data_point.x) ** 2
        + (second_data_point.y - first_data_point.y) ** 2
    )


def get_angle_between_data_points(first_data_point, second_data_point):
    first_angle = first_data_point.angle
    if first_angle < 0:
        first_angle = 2 * math.pi + first_angle
    second_angle = second_data_point.angle
    if second_angle < 0:
        second_angle = 2 * math.pi + second_angle
    return abs(Decimal(first_angle) - Decimal(second_angle))


def get_distance_to_line(segment: Object, point: Point):
    left = segment.get_boundary(left=True)
    right = segment.get_boundary()

    x2 = left.range * math.cos(left.angle)
    y2 = left.range * math.sin(left.angle)
    x1 = right.range * math.cos(right.angle)
    y1 = right.range * math.sin(right.angle)
    x0 = point.range * math.cos(point.angle)
    y0 = point.range * math.sin(point.angle)

    length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    if length == 0:
        return 0

    d = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / length

    return d


def are_same_point(point1: Point, point2: Optional[Point]):
    if isinstance(point1, Point) and isinstance(point2, Point):
        return point1 == point2
    return False


def get_numpy_array_from_points(points: List[Point]):
    array = [(point.x, point.y) for point in points]
    return np.array(array)


def get_points_from_numpy_array(numpy_array):
    points = [
        Point.init_from_rectangular(x=point[0], y=point[1]) for point in numpy_array
    ]
    points.sort(key=lambda x: x.angle)
    return points
