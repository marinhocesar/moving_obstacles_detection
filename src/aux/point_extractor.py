from typing import List

from components.object import Object
from components.point import Point
from aux.global_config import GlobalConfig


def get_real_significant_points(segments: List[Object], all_points: List[Point]):

    max_range = GlobalConfig.NEIGHBORHOOD_CHECK_FOR_APPARENT_SIGNIFICANT

    for segment in segments:
        real_significant_points = []
        apparent_significant_points = []
        # check points to the left of the boundaries
        left_outline_point = segment.get_boundary(left=True)

        # find boundary in all_points
        l_boundary_idx = 0
        for idx, point in enumerate(all_points):
            if point.angle == left_outline_point.angle:
                l_boundary_idx = idx
                break

        h_left = [
            all_points[(l_boundary_idx - j) % len(all_points)] for j in range(max_range)
        ]

        is_real_significant = False
        for point in h_left:
            if point.range > left_outline_point.range:
                is_real_significant = True
                break
        
        if is_real_significant is True:
            # if left_outline_point in real_significant_points:
            #     continue
            real_significant_points.append(left_outline_point)
        else:
            # if left_outline_point in apparent_significant_points:
            #     continue
            apparent_significant_points.append(left_outline_point)

        # check points to the right of the boundaries
        right_outline_point = segment.get_boundary(left=False)

        # find boundary in all_points
        r_boundary_idx = 0
        for idx, point in enumerate(all_points):
            if point.angle == right_outline_point.angle:
                r_boundary_idx = idx
                break

        h_right = [
            all_points[(r_boundary_idx + j) % len(all_points)] for j in range(max_range)
        ]

        is_real_significant = False
        for point in h_right:
            if point.range > right_outline_point.range:
                is_real_significant = True
                real_significant_points.append(right_outline_point)
                break

        if is_real_significant is True:
            # if right_outline_point in real_significant_points:
            #     continue
            real_significant_points.append(right_outline_point)
        else:
            # if right_outline_point in apparent_significant_points:
            #     continue
            apparent_significant_points.append(right_outline_point)

        for point in segment.significant_inside:
            if point not in real_significant_points:
                real_significant_points.append(point)

        segment.register_real_significant_points(
            real_significant_points=real_significant_points
        )
        segment.register_apparent_significant_points(
            apparent_significant_points=apparent_significant_points
        )
