import math
from typing import List, Optional

from aux import aux_functions as aux
from aux.global_config import GlobalConfig
from components.object import Object
from components.point import Point


def get_angle_for_exclusion(first_data_point, second_data_point):
    l_1 = first_data_point.range
    l_2 = second_data_point.range
    angle_between = aux.get_angle_between_data_points(
        first_data_point=first_data_point, second_data_point=second_data_point
    )

    if angle_between == 0:
        #  limit when angle -> 0
        return (
            ((l_2 - l_1) / math.sqrt(l_1**2 + l_2**2 - l_1 * l_2)) * 180 / math.pi
        )

    acos_argument = (l_2 - l_1 * math.cos(angle_between)) / (
        math.sqrt(l_1**2 + l_2**2 - 2 * l_1 * l_2 * math.cos(angle_between))
    )

    return math.acos(acos_argument) * 180 / math.pi  # returns in degrees


def should_be_excluded(current, last_in_object, last_evaluated):
    if last_in_object is None:
        return False

    beta_1 = get_angle_for_exclusion(
        first_data_point=last_in_object,
        second_data_point=last_evaluated,
    )
    beta_2 = get_angle_for_exclusion(
        first_data_point=last_evaluated,
        second_data_point=current,
    )

    return abs(beta_1 - beta_2) < GlobalConfig.EXCLUSION_THRESHOLD


def join_segments_that_have_close_boundaries(segments: List[Object]) -> List[Object]:
    if GlobalConfig.SHOULD_JOIN is False:
        return segments

    pairs = dict()
    for idx1, segment1 in enumerate(segments):
        for idx2, segment2 in enumerate(segments):
            if idx1 == idx2:
                continue

            if idx2 in pairs.values():
                continue

            if segment1.is_boundary_close(segment2):
                pairs[idx1] = idx2

    joined = list()
    for key in pairs:
        first = segments[key]
        second = segments[pairs[key]]
        first_points = first.points
        second_points = second.points

        if first.center.angle - second.center.angle > 0:
            first_and_second = first_points + second_points
        else:
            first_and_second = second_points + first_points

        new_segment = Object(points=first_and_second)
        joined.append(new_segment)

    for idx, segment in enumerate(segments):
        if idx not in pairs.keys() and idx not in pairs.values():
            joined.append(segment)

    return joined


def segment_lidar_data(scan_points: List[Point]):
    segments = []
    segment = []
    last_evaluated_point = scan_points[0]
    last_in_object: Optional[Point] = None
    skipped_points = 0
    for current_point in scan_points:
        distance_between_last_and_current_point = aux.get_distance_between_data_points(
            last_evaluated_point, current_point
        )

        if current_point.range == 0 or current_point.range == -0:
            # ignore points that have range equal to zero (exactly)
            continue

        if current_point.range < GlobalConfig.MIN_RANGE:
            # ignore points that have range close to zero
            continue

        # compares to the last range measurement for the same angle
        # current_point = compare_to_previous_measurement(previous_points=previous_points, last_evaluated_point=current_point)

        if current_point.range >= GlobalConfig.MAX_RANGE:
            # current_point = Point(range=float("Inf"), angle=current_point.angle)
            continue

        if (
            distance_between_last_and_current_point
            > GlobalConfig.SEGMENTATION_THRESHOLD
        ):
            """
            uses the distance threshold criteria to determine if two points
            are part of the same segment.
            if the distance is greater than the threshold, the current segment
            is closed and a new segment is created.
            """

            if bool(segment) is not False:
                if not aux.are_same_point(last_evaluated_point, last_in_object):
                    # checks if the last evaluated was included in the previous
                    # segment before closing it and creating the next one

                    segment.append(last_evaluated_point)  # INCLUDES

                # CLOSES SEGMENT
                closed_segment = Object(points=segment)
                segments.append(closed_segment)

            # OPENS NEW SEGMENT

            segment = [current_point]  # INCLUDES
            last_in_object = current_point
            skipped_points = 0

        else:
            if aux.are_same_point(last_evaluated_point, last_in_object):
                last_evaluated_point = current_point
                continue

            if skipped_points < GlobalConfig.MAX_SKIPPED and should_be_excluded(
                current=current_point,
                last_evaluated=last_evaluated_point,
                last_in_object=last_in_object,
            ):
                skipped_points += 1
                pass
            else:
                segment.append(current_point)  # INCLUDES
                last_in_object = current_point
                skipped_points = 0

        last_evaluated_point = current_point

    return segments
