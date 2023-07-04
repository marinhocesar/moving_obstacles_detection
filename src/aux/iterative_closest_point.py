import math
from typing import List, Tuple

# from aux.icp import icp

from aux.global_config import GlobalConfig
from components.buffer import SignificantBuffer
from aux import aux_functions as aux
from components.point import Point


# def get_displacement_for_significant_points(
#     point_cloud_history: SignificantBuffer,
# ) -> "Tuple[dict[Point, Point], dict[Point, Point]]":
#     buffer_size = point_cloud_history.size
#     current_record_n = point_cloud_history.last_record
#     current_record_idx = current_record_n % buffer_size
#     all_real_significant = point_cloud_history.real_significant
#     all_apparent_significant = point_cloud_history.apparent_significant
#     last_real_significant = all_real_significant[current_record_idx]
#     last_apparent_significant = all_apparent_significant[current_record_idx]
#     interval_size = GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT

#     if current_record_n < interval_size:
#         n_records = current_record_n
#     else:
#         n_records = interval_size

#     displacement_real_significant = {
#         point: Point(0, 0) for point in last_real_significant
#     }
#     displacement_apparent_significant = {
#         point: Point(0, 0) for point in last_apparent_significant
#     }

#     if current_record_n == 0:
#         no_displacement_real = {
#             point: Point(range=0, angle=0) for point in last_real_significant
#         }
#         no_displacement_apparent = {
#             point: Point(range=0, angle=0) for point in last_apparent_significant
#         }
#         return no_displacement_real, no_displacement_apparent

#     idx = (current_record_idx - n_records + 1) % buffer_size
#     compare_real_sig = all_real_significant[idx]
#     compare_apparent_sig = all_apparent_significant[idx]

#     A_real = aux.get_numpy_array_from_points(
#         points=last_real_significant + last_apparent_significant
#     )
#     B_real = aux.get_numpy_array_from_points(
#         points=compare_real_sig + compare_apparent_sig
#     )

#     history, C_real = icp(A_real, B_real, point_pairs_threshold=10)

#     compare_real_sig = aux.get_points_from_numpy_array(numpy_array=C_real)

#     for point in last_real_significant:
#         distances_real_sig = [
#             aux.get_distance_between_data_points(point, other_point)
#             for other_point in (compare_real_sig + compare_apparent_sig)
#         ]
#         closest_real_point = (compare_real_sig + compare_apparent_sig)[
#             distances_real_sig.index(min(distances_real_sig))
#         ]

#         displacement = get_displacement(
#             first_point=point,
#             second_point=closest_real_point,
#         )

#         displacement_real_significant[point] = displacement

#     # evaluating apparent significant points
#     for point in last_apparent_significant:
#         distances_real_sig = [
#             aux.get_distance_between_data_points(point, other_point)
#             for other_point in (compare_real_sig + compare_apparent_sig)
#         ]
#         closest_real_point = (compare_real_sig + compare_apparent_sig)[
#             distances_real_sig.index(min(distances_real_sig))
#         ]

#         displacement = get_displacement(
#             first_point=point,
#             second_point=closest_real_point,
#         )

#         displacement_apparent_significant[point] = displacement

#     return displacement_real_significant, displacement_apparent_significant


def get_displacement_for_significant_points(
    point_cloud_history: SignificantBuffer,
) -> "Tuple[dict[Point, Point], dict[Point, Point]]":
    buffer_size = point_cloud_history.size
    current_record_n = point_cloud_history.last_record
    current_record_idx = current_record_n % buffer_size
    all_real_significant = point_cloud_history.real_significant
    all_apparent_significant = point_cloud_history.apparent_significant
    last_real_significant = all_real_significant[current_record_idx]
    last_apparent_significant = all_apparent_significant[current_record_idx]
    interval_size = GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT

    if current_record_n < interval_size:
        n_records = current_record_n
    else:
        n_records = interval_size

    displacement_real_significant = {
        point: Point(0, 0) for point in last_real_significant
    }
    displacement_apparent_significant = {
        point: Point(0, 0) for point in last_apparent_significant
    }

    if current_record_n == 0:
        no_displacement_real = {
            point: Point(range=0, angle=0) for point in last_real_significant
        }
        no_displacement_apparent = {
            point: Point(range=0, angle=0) for point in last_apparent_significant
        }
        return no_displacement_real, no_displacement_apparent

    idx = (current_record_idx - n_records + 1) % buffer_size
    compare_real_sig = all_real_significant[idx]
    compare_apparent_sig = all_apparent_significant[idx]

    if len(compare_real_sig) == 0 and len(compare_apparent_sig) == 0:
        return displacement_real_significant, displacement_apparent_significant

    # evaluating real significant points
    for point in last_real_significant:
        distances_real_sig = [
            aux.get_distance_between_data_points(point, other_point)
            for other_point in compare_real_sig
        ]
        if len(distances_real_sig) == 0:
            distances_real_sig = [0.0]
            compare_real_sig = [point]

        distances_apparent_sig = [
            aux.get_distance_between_data_points(point, other_point)
            for other_point in compare_apparent_sig
        ]
        if len(distances_apparent_sig) == 0:
            distances_apparent_sig = [0.0]
            compare_apparent_sig = [point]

        closest_real_point = compare_real_sig[
            distances_real_sig.index(min(distances_real_sig))
        ]
        closest_apparent_point = compare_apparent_sig[
            distances_apparent_sig.index(min(distances_apparent_sig))
        ]

        if min(distances_real_sig) < min(distances_apparent_sig):
            displacement = get_displacement(
                first_point=point,
                second_point=closest_real_point,
            )
        else:
            displacement = get_displacement(
                first_point=point,
                second_point=closest_apparent_point,
            )

        displacement_real_significant[point] = displacement

    # evaluating apparent significant points
    for point in last_apparent_significant:
        distances_real_sig = [
            aux.get_distance_between_data_points(point, other_point)
            for other_point in compare_real_sig
        ]
        if len(distances_real_sig) == 0:
            distances_real_sig = [0.0]
            compare_real_sig = [point]

        distances_apparent_sig = [
            aux.get_distance_between_data_points(point, other_point)
            for other_point in compare_apparent_sig
        ]
        if len(distances_apparent_sig) == 0:
            distances_apparent_sig = [0.0]
            compare_apparent_sig = [point]

        closest_real_point = compare_real_sig[
            distances_real_sig.index(min(distances_real_sig))
        ]
        closest_apparent_point = compare_apparent_sig[
            distances_apparent_sig.index(min(distances_apparent_sig))
        ]

        if min(distances_real_sig) < min(distances_apparent_sig):
            displacement = get_displacement(
                first_point=point,
                second_point=closest_real_point,
            )
        else:
            displacement = get_displacement(
                first_point=point,
                second_point=closest_real_point,
            )

        displacement_apparent_significant[point] = displacement

    return displacement_real_significant, displacement_apparent_significant


def get_avg_displacement_for_significant_points(
    point_cloud_history: SignificantBuffer,
) -> "Tuple[dict[Point, Point], dict[Point, Point]]":
    buffer_size = point_cloud_history.size
    current_record_n = point_cloud_history.last_record
    current_record_idx = current_record_n % buffer_size
    all_real_displacement = point_cloud_history.displacement_real
    all_apparent_displacement = point_cloud_history.displacement_apparent
    last_real_significant = all_real_displacement[current_record_idx]
    last_apparent_significant = all_apparent_displacement[current_record_idx]
    if current_record_n < buffer_size:
        n_records = current_record_n
    else:
        n_records = buffer_size

    displ_real: dict[Point, List[Point]] = {
        point: list() for point in last_real_significant
    }
    displ_apparent: dict[Point, List[Point]] = {
        point: list() for point in last_apparent_significant
    }

    if current_record_n < 1:
        no_displacement_real = {
            point: Point(range=0, angle=0) for point in last_real_significant
        }
        no_displacement_apparent = {
            point: Point(range=0, angle=0) for point in last_apparent_significant
        }
        return no_displacement_real, no_displacement_apparent

    for i in range(n_records + 1):
        idx = (current_record_idx - i) % buffer_size
        compare_real_displ = all_real_displacement[idx]
        compare_apparent_displ = all_apparent_displacement[idx]
        compare = {**compare_real_displ, **compare_apparent_displ}

        if len(compare) == 0:
            empty_displ = dict()
            return empty_displ, empty_displ

        for point in displ_real.keys():
            closest_point = get_closest_point(
                point=point,
                points_to_compare=list(compare),
            )

            displacement = get_displacement(point, closest_point)

            displ_real[point].append(displacement)

        for point in displ_apparent.keys():
            closest_point = get_closest_point(
                point=point,
                points_to_compare=list(compare),
            )
            displacement = get_displacement(point, closest_point)

            displ_apparent[point].append(displacement)

    avg_displ_real = {point: Point(0, 0) for point in displ_real}
    avg_displ_apparent = {point: Point(0, 0) for point in displ_apparent}

    for point in displ_real.keys():
        avg_displ_real[point] = get_avg_displacement(displacements=displ_real[point])
    for point in displ_apparent.keys():
        avg_displ_apparent[point] = get_avg_displacement(
            displacements=displ_apparent[point]
        )

    return avg_displ_real, avg_displ_apparent


def get_displacement(
    first_point: Point,
    second_point: Point,
) -> Point:
    d = aux.get_distance_between_data_points(
        first_data_point=first_point, second_data_point=second_point
    )
    if d > GlobalConfig.MAX_MATCHING_DISTANCE:
        return Point(0, 0)

    x_coord = first_point.x - second_point.x
    y_coord = first_point.y - second_point.y
    return Point.init_from_rectangular(x=x_coord, y=y_coord)


def get_avg_displacement(displacements: List[Point]) -> Point:
    no_displacement = Point(0, 0)
    real_displacements = [
        disp
        for disp in displacements
        if disp.range > GlobalConfig.DISPLACEMENT_THRESHOLD
    ]
    # real_displacements = [disp for disp in displacements]
    n = len(real_displacements)
    if n < 1:
        return no_displacement

    x_pos = [point.x for point in real_displacements]
    y_pos = [point.y for point in real_displacements]
    avg_x = sum(x_pos) / n
    avg_y = sum(y_pos) / n
    v_x = math.sqrt(sum([(xi - avg_x) ** 2 for xi in x_pos]) / n)
    v_y = math.sqrt(sum([(yi - avg_y) ** 2 for yi in y_pos]) / n)

    if v_x < GlobalConfig.VARIANCE_THRESHOLD and v_y < GlobalConfig.VARIANCE_THRESHOLD:
        return no_displacement

    avg_disp = Point.init_from_rectangular(x=avg_x, y=avg_y)

    if avg_disp.range < GlobalConfig.DISPLACEMENT_THRESHOLD:
        return no_displacement

    return avg_disp


def get_closest_point(point: Point, points_to_compare: List[Point]) -> Point:
    distances_to_point = [
        aux.get_distance_between_data_points(point, other_point)
        for other_point in points_to_compare
    ]
    closest_point_index = distances_to_point.index(min(distances_to_point))
    closest = points_to_compare[closest_point_index]
    return closest


def get_second_closest_point(
    point: Point, closest_index: int, points_to_compare: List[Point]
) -> Point:
    new_points = points_to_compare.copy()
    new_points.pop(closest_index)

    distances_to_point = [
        aux.get_distance_between_data_points(point, other_point)
        for other_point in new_points
    ]
    second_closest_point_index = distances_to_point.index(min(distances_to_point))
    second_closest = new_points[second_closest_point_index]
    return second_closest
