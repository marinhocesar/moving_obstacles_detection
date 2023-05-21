import math
from scipy import optimize
from typing import List, Optional

from aux.global_config import GlobalConfig

from .point import Point
from aux import aux_functions as aux


class Object:
    def __init__(self, points: List[Point]):
        self.points = points
        self.n: int = len(points)
        self.x_pos: List[float] = [
            point.range * math.cos(point.angle) for point in points
        ]
        self.y_pos: List[float] = [
            point.range * math.sin(point.angle) for point in points
        ]
        self.avg_x: float = sum(self.x_pos) / self.n
        self.avg_y: float = sum(self.y_pos) / self.n
        self.std_dx: float = math.sqrt(
            sum([(xi - self.avg_x) ** 2 for xi in self.x_pos]) / self.n
        )
        self.std_dy: float = math.sqrt(
            sum([(yi - self.avg_y) ** 2 for yi in self.y_pos]) / self.n
        )
        self.a = 1.5 * self.std_dx
        self.b = 1.5 * self.std_dy
        self.distance_to_center = math.sqrt(self.avg_x**2 + self.avg_y**2)
        self.left_boundary: Point = self.get_boundary(left=True)
        self.right_boundary: Point = self.get_boundary()
        self.real_significant_points: List[Point] = list()
        self.apparent_significant_points: List[Point] = list()
        self.distances: List[float] = self.get_distances()
        self.min_length: float = 0 if len(self.distances) == 0 else min(self.distances)
        self.significant_inside: List[Point] = self.get_inside_significant()
        self.center: Point = Point(
            range=self.distance_to_center, angle=math.atan2(self.avg_y, self.avg_x)
        )
        self.a_coeff, self.b_coeff, self.c_coeff = None, None, None
        self.y_line = list()
        if self.n > 1:
            params, extra = optimize.curve_fit(
                f=aux.linear_function,
                xdata=self.x_pos,
                ydata=self.y_pos,
            )
            self.a_coeff, self.c_coeff = params
            self.b_coeff = -1
            self.y_line = [
                aux.linear_function(x=x, a=self.a_coeff, b=self.c_coeff)
                for x in self.x_pos
            ]

        self.distance_to_fitting = [
            self.get_distance_to_fitting_line(point=point) for point in self.points
        ]
        self.real_significant_displacements: "dict[Point, Point]" = dict()
        self.apparent_significant_displacements: "dict[Point, Point]" = dict()
        self.avg_displacement: Optional[Point] = None
        self.velocity_x: List[float] = list()
        self.velocity_y: List[float] = list()

    def register_displacement_real_significant_points(
        self, real_displacements: "dict[Point, Point]"
    ) -> None:
        self.real_significant_displacements = real_displacements

    def register_displacement_apparent_significant_points(
        self, apparent_displacements: "dict[Point, Point]"
    ) -> None:
        self.apparent_significant_displacements = apparent_displacements

    def set_velocity_vector_for_display(self) -> None:
        if self.avg_displacement is None:
            print("No average displacement has been set.")
            return
        if self.avg_displacement.range == 0:
            self.velocity_x = []
            self.velocity_y = []
            return

        normalized_displacement = self.avg_displacement.normalize()
        scaled_displacement = normalized_displacement.scale(
            0.05 * GlobalConfig.VECTOR_SCALING
        )

        velocity_vector = Point.init_from_rectangular(
            x=self.center.x + scaled_displacement.x,
            y=self.center.y + scaled_displacement.y,
        )

        x = [self.center.x, velocity_vector.x]
        y = [self.center.y, velocity_vector.y]
        params, extra = optimize.curve_fit(f=aux.linear_function, xdata=x, ydata=y)

        a_coeff, b_coeff = params

        self.velocity_x = x
        self.velocity_y = [
            aux.linear_function(x=x, a=a_coeff, b=b_coeff) for x in self.velocity_x
        ]

    def set_average_displacement(self) -> None:
        real_displacements = self.real_significant_displacements
        apparent_displacements = self.apparent_significant_displacements
        n = len(real_displacements) + len(apparent_displacements)

        if n == 0:
            return

        x_coord = (
            sum(
                [
                    real_displacements[displacement].x
                    for displacement in real_displacements
                    if real_displacements[displacement]
                ]
                + [
                    apparent_displacements[displacement].x
                    for displacement in apparent_displacements
                ]
            )
            / n
        )
        y_coord = (
            sum(
                [
                    real_displacements[displacement].y
                    for displacement in real_displacements
                ]
                + [
                    apparent_displacements[displacement].y
                    for displacement in apparent_displacements
                ]
            )
            / n
        )

        self.avg_displacement = Point.init_from_rectangular(x=x_coord, y=y_coord)

    def get_distance_to_fitting_line(self, point: Point) -> float:
        if self.a_coeff is None or self.b_coeff is None or self.c_coeff is None:
            return 0

        x = point.range * math.cos(point.angle)
        y = point.range * math.sin(point.angle)
        d = abs(self.a_coeff * x + self.b_coeff * y + self.c_coeff) / math.sqrt(
            self.a_coeff**2 + self.b_coeff**2
        )
        return d

    def get_distance_to_predicted_point(self, point) -> float:
        if self.a_coeff is None or self.b_coeff is None or self.c_coeff is None:
            return 0

        theta = point.angle
        predicted_point_x = -(self.c_coeff * math.cos(theta)) / (
            self.a_coeff * math.cos(theta) + self.b_coeff * math.sin(theta)
        )
        predicted_point_y = -(self.c_coeff * math.sin(theta)) / (
            self.a_coeff * math.cos(theta) + self.b_coeff * math.sin(theta)
        )

        predicted_point = Point(
            range=math.sqrt(predicted_point_x**2 + predicted_point_y**2),
            angle=math.atan2(predicted_point_y, predicted_point_x),
        )

        return aux.get_distance_between_data_points(point, predicted_point)

    def get_avg_significant(self):
        n = len(self.significant_inside)
        if n < 2:
            return self.significant_inside

        significant_cartesian = [
            (point.range * math.cos(point.angle), point.range * math.sin(point.angle))
            for point in self.significant_inside
        ]

        avg_x = sum([point[0] for point in significant_cartesian]) / n
        avg_y = sum([point[1] for point in significant_cartesian]) / n

        avg_r = math.sqrt(avg_x**2 + avg_y**2)
        avg_angle = math.atan2(avg_y, avg_x)

        return [Point(range=avg_r, angle=avg_angle)]

    def register_real_significant_points(
        self, real_significant_points: List[Point]
    ) -> None:
        points_dict = dict.fromkeys(real_significant_points, "")
        for point in points_dict.keys():
            self.real_significant_points.append(point)

    def register_apparent_significant_points(
        self, apparent_significant_points: List[Point]
    ) -> None:
        points_dict = dict.fromkeys(apparent_significant_points, "")
        for point in points_dict.keys():
            self.apparent_significant_points.append(point)

    def get_boundary(self, left=False):
        if self.points[0].angle > self.points[-1].angle:
            if left:
                return self.points[0]
            return self.points[-1]
        else:
            if left:
                return self.points[-1]
            return self.points[0]

    def get_inside_significant(self):
        significant = []
        for idx, point in enumerate(self.points):
            if idx == 0 or idx == self.n - 1:
                continue
            if self.is_internal_significant(i=idx, k=0):
                significant.append(point)

        if len(significant) == 0:
            significant = self.get_eccentric_significant()

        return significant

    def get_furthermost_from_border(self, points):
        curve_length = 0
        for i in range(len(points) - 1):
            curve_length += aux.get_distance_between_data_points(
                points[i], points[i + 1]
            )

        if curve_length == 0:
            return

        distances = []
        for point in points:
            start_distance = aux.get_distance_between_data_points(points[0], point)
            end_distance = aux.get_distance_between_data_points(points[-1], point)

            border_distance = min(start_distance, end_distance)

            normalized_distance = border_distance / curve_length

            distances.append(normalized_distance)

        furthest_index = distances.index(max(distances))

        return points[furthest_index]

    def get_eccentric_significant(self):
        significant_points = []
        if len(self.points) == 1:
            return significant_points

        point = self.get_furthermost_from_border(self.points)
        if point is None:
            return significant_points

        left_b = self.get_boundary(left=True)
        right_b = self.get_boundary(left=False)

        length = aux.get_distance_between_data_points(left_b, right_b)

        distance_to_line = aux.get_distance_to_line(segment=self, point=point)
        threshold = GlobalConfig.ECCENTRICITY_THRESHOLD_COEFFICIENT * length

        if distance_to_line > threshold:
            significant_points.append(point)

        return significant_points

    def is_boundary_close(self, other_segment):
        # checks is the boundaries of a segment are next to the boundaries of other segment

        xb = [other_segment.x_pos[0], other_segment.x_pos[-1]]
        yb = [other_segment.y_pos[0], other_segment.y_pos[-1]]

        if self.a == 0 or self.b == 0:
            return False

        for i in range(2):
            first_b = ((xb[i] - self.x_pos[0]) / self.a) ** 2 + (
                (yb[i] - self.y_pos[0]) / self.b
            ) ** 2 < 1
            second_b = ((xb[i] - self.x_pos[-1]) / self.a) ** 2 + (
                (yb[i] - self.y_pos[-1]) / self.b
            ) ** 2 < 1
            if first_b or second_b:
                return True

        return False

    def is_internal_significant(self, i, k):
        j = i + 1 + k
        points = self.points

        if j > self.n - 1:
            return False

        point_i = points[i]
        point_i_minus_one = points[i - 1]
        point_j = points[j]

        angle = self.get_angles_between_line_segments(i=i, j=j)
        angle = angle * (180 / math.pi)

        if abs(point_i.range - point_i_minus_one.range) < self.min_length:
            return False
        if (
            abs(point_i.range - point_i_minus_one.range) > self.min_length
            and abs(point_i.range - point_j.range) > self.min_length
            and (angle < GlobalConfig.MIN_ANGLE or angle > GlobalConfig.MAX_ANGLE)
        ):
            return False
        if (
            abs(point_i.range - point_i_minus_one.range) > self.min_length
            and abs(point_i.range - point_j.range) < self.min_length
        ):
            return self.is_internal_significant(i=i, k=k + 1)
        if (
            abs(point_i.range - point_i_minus_one.range) > self.min_length
            and abs(point_i.range - point_j.range) > self.min_length
            and (angle >= GlobalConfig.MIN_ANGLE and angle <= GlobalConfig.MAX_ANGLE)
        ):
            # return False  # temporarily set to False for the testing of internal significant points based on eccentricity criteria
            return True

        return False

    def get_angles_between_line_segments(self, i, j):
        point: Point = self.points[i]
        prev: Point = self.points[i - 1]
        after: Point = self.points[j]

        point_x, point_y = point.get_rectangular()
        prev_x, prev_y = prev.get_rectangular()
        after_x, after_y = after.get_rectangular()

        m1, m2 = 0, 0
        if point_x - prev_x != 0:
            m1 = (point_y - prev_y) / (point_x - prev_x)
        if after_x - point_x != 0:
            m2 = (after_y - point_y) / (after_x - point_x)

        tan = (m2 - m1) / (1 + m1 * m2)
        angle_between = math.atan(tan)

        return angle_between

    def get_distances(self):
        distances = []
        for idx, point in enumerate(self.points):
            if idx == 0:
                continue

            prev_point = self.points[idx - 1]
            d = aux.get_distance_between_data_points(prev_point, point)
            # d = point.range - prev_point.range
            distances.append(abs(d))

        return distances

    def get_length(self):
        return aux.get_distance_between_data_points(
            self.left_boundary, self.right_boundary
        )
