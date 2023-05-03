from aux.global_config import GlobalConfig
from components.point import Point


def compare_to_previous_measurement(previous_points, last_evaluated_point):
    point = last_evaluated_point
    truncated_angle = get_truncated_angle(point.angle)
    previous_range_at_angle = previous_points.get(truncated_angle, None)

    if previous_range_at_angle is not None:
        diff = abs(previous_range_at_angle - point.range)

        if diff / point.range < GlobalConfig.MAX_ALLOWED_MEASURE_DIFF:
            print(diff / point.range)
            print(point.angle, point.range)
            point = Point(
                range=previous_range_at_angle,
                angle=truncated_angle / GlobalConfig.ROUNDING_FACTOR,
            )
            print(point.angle, point.range)

    return point


def get_truncated_angle(angle):
    return round(angle * GlobalConfig.ROUNDING_FACTOR)


def get_points_from_segments(segments, dict):
    for segment in segments:
        for point in segment.points:
            truncated_angle = get_truncated_angle(point.angle)
            dict[truncated_angle] = point.range

    return dict


def filter_data(data):
    """Placeholder para quando implementar filtro"""
    return data


# def kalman_filter(data):
#     # Define the Kalman Filter model
#     kf = KalmanFilter(n_dim_obs=2, n_dim_state=2)

#     # Apply the filter to the data
#     filtered_data = []
#     for i in range(len(data)):
#         if i == 0:
#             filtered_state_mean = np.array([data[0].range, data[0].angle])
#             filtered_state_covariance = np.eye(2)
#         else:
#             observation = np.array([data[i].range, data[i].angle])
#             filtered_state_mean, filtered_state_covariance = kf.filter_update(
#                 filtered_state_mean, filtered_state_covariance, observation=observation)
#         filtered_data.append(filtered_state_mean)

#     filtered_points = [Point(range=point[0], angle=point[1]) for point in filtered_data]
#     return filtered_points
