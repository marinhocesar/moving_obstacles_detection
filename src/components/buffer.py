from typing import List
from aux.global_config import GlobalConfig

from components.point import Point


class SignificantBuffer:
    def __init__(self, size: int):
        self.size: int = size
        self.real_significant: List[List[Point]] = [list()] * self.size
        self.apparent_significant: List[List[Point]] = [list()] * self.size
        self.lidar: List[List[Point]] = [list()] * self.size
        self.displacement_real: List[dict[Point, Point]] = [dict()] * self.size
        self.displacement_apparent: List[dict[Point, Point]] = [dict()] * self.size
        self.time: List[float] = [0] * self.size
        self.last_record: int = -1

    def register_displacement(
        self, real: "dict[Point, Point]", apparent: "dict[Point, Point]"
    ) -> None:
        self.__register_real_displacement(real)
        self.__register_apparent_displacement(apparent)

    def __register_real_displacement(self, real: "dict[Point, Point]") -> None:
        index = self.last_record % self.size
        self.displacement_real[index] = real

    def __register_apparent_displacement(self, apparent: "dict[Point, Point]") -> None:
        index = self.last_record % self.size
        self.displacement_apparent[index] = apparent

    def register_significant(
        self, real: List[Point], apparent: List[Point], lidar: List[Point]
    ) -> None:
        self.last_record += 1
        index = self.last_record % self.size

        self.time[index] = self.time[index-1] + 1 / GlobalConfig.LIDAR_FREQUENCY

        self.__register_real_significant(points_to_save=real, record_position=index)
        self.__register_apparent_significant(
            points_to_save=apparent, record_position=index
        )
        self.__register_lidar(
            points_to_save=lidar, record_position=index
        )

    def __remove_duplicate_points(self, points: List[Point]):
        # somewhere in the code, significant points are being registered multiple times
        # as a workaround, I implemented a method to remove duplicates
        points_dict = {point for point in points}
        return list(points_dict)

    def __register_lidar(
        self, points_to_save: List[Point], record_position: int
    ) -> None:
        points = self.__remove_duplicate_points(points_to_save)
        self.lidar[record_position] = points

    def __register_real_significant(
        self, points_to_save: List[Point], record_position: int
    ) -> None:
        points = self.__remove_duplicate_points(points_to_save)
        self.real_significant[record_position] = points

    def __register_apparent_significant(
        self, points_to_save: List[Point], record_position: int
    ) -> None:
        points = self.__remove_duplicate_points(points_to_save)
        self.apparent_significant[record_position] = points

    def get_time_between(self):
        last_time_idx = self.last_record % self.size
        last_time = self.time[last_time_idx]
        if self.last_record < GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT:
            previous_idx = 0
        else:
            previous_idx = (self.last_record - GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT + 1) % self.size
        previous_time = self.time[previous_idx]

        return last_time - previous_time

    def get_lidar(self):
        last_time_idx = self.last_record % self.size
        lidar = self.lidar[last_time_idx]
        return lidar
