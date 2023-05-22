import math
import time
from typing import List
from matplotlib.axes import Axes
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Circle, Rectangle
import rospy
from sensor_msgs.msg import LaserScan
from moving_obstacles_detection.msg import MovingObstacles
from aux.icp import icp
from aux import aux_functions as aux


from aux.iterative_closest_point import (
    get_avg_displacement_for_significant_points,
    get_displacement_for_significant_points,
)
from components.buffer import SignificantBuffer
from components.point import Point
from components.object import Object
from aux.global_config import GlobalConfig
from aux import split_and_merge, point_extractor
import warnings

warnings.simplefilter("ignore")


class Publisher:
    def __init__(self) -> None:
        self.pub = rospy.Publisher("/obstacles", MovingObstacles, queue_size=1)


class SimpleSubscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)
        self.points = list()

    def callback(self, msg: LaserScan):
        ranges = msg.ranges
        if len(ranges) == 0:
            return
        self.points = [
            Point(range=ranges[i], angle=msg.angle_min + msg.angle_increment * i)
            for i in range(0, len(ranges))
        ]

    def printMsg(self):
        print(self.points)

    def get_points(self) -> List[Point]:
        return self.points


def animate(
    scan: List[Point],
    significant_buffer: SignificantBuffer,
    lidar: Axes,
    publisher: Publisher,
):
    if not scan:
        return

    lidar.clear()
    lidar.autoscale(False)
    plt.grid(True)
    ax = plt.gca()
    ax.set_xlim([-GlobalConfig.MAX_RANGE, GlobalConfig.MAX_RANGE])
    ax.set_ylim([-GlobalConfig.MAX_RANGE, GlobalConfig.MAX_RANGE])
    plt.gca().set_aspect("equal")
    cmap = cm.get_cmap(
        "coolwarm"
    )  # to see how many points in each segment, ordered by distance to sensor:

    # configure data for visualization
    structured_data = [Point(range=point.range, angle=point.angle) for point in scan]
    if significant_buffer.last_record > 1:
        reference_points = aux.get_numpy_array_from_points(
            points=significant_buffer.get_lidar()
        )
    else:
        reference_points = aux.get_numpy_array_from_points(points=structured_data)
    points = aux.get_numpy_array_from_points(points=structured_data)
    history, structured_data = icp(reference_points=reference_points, points=points)
    structured_data = aux.get_points_from_numpy_array(numpy_array=structured_data)

    segments = split_and_merge.segment_lidar_data(scan_points=structured_data)
    joined_segments = split_and_merge.join_segments_that_have_close_boundaries(
        segments=segments
    )

    distances = [seg.distance_to_center for seg in joined_segments]
    max_d = GlobalConfig.MAX_RANGE
    if distances:
        max_d = max(distances)

    point_extractor.get_real_significant_points(
        segments=joined_segments, all_points=structured_data
    )

    # plots the position of the sensor
    lidar.plot(0, 0, "x", color="green")

    all_real_significant: List[Point] = list()
    all_apparent_significant: List[Point] = list()

    for segment in joined_segments:
        # plot the basic points that form each obstacle
        lidar.plot(
            segment.x_pos,
            segment.y_pos,
            "-",
            linewidth=2,
            color=cmap(segment.distance_to_center / max_d),
        )

        if segment.y_line and GlobalConfig.SHOULD_RENDER_FIT_CURVE:
            # renders the curve that best fits the point distribution for a particular obstacle
            lidar.plot(
                segment.x_pos,
                segment.y_line,
                "-",
                linewidth=1,
                color="red",
            )

        if segment.real_significant_points:
            real_significant = Object(points=segment.real_significant_points)
            all_real_significant += real_significant.points
            lidar.scatter(real_significant.x_pos, real_significant.y_pos, color="red")  # type: ignore

        if segment.apparent_significant_points:
            apparent_significant = Object(points=segment.apparent_significant_points)
            all_apparent_significant += apparent_significant.points
            lidar.scatter(
                apparent_significant.x_pos,
                apparent_significant.y_pos,
                color="blue",
                alpha=0.2,
            )

        # Plot center of object
        center_x, center_y = segment.center.get_rectangular()
        lidar.plot(center_x, center_y, "x", color="purple")

    significant_buffer.register_significant(
        real=all_real_significant,
        apparent=all_apparent_significant,
        lidar=structured_data,
    )

    # get displacements
    (
        displacement_real_significant,
        displacement_apparent_significant,
    ) = get_displacement_for_significant_points(point_cloud_history=significant_buffer)
    # register displacements to buffer
    significant_buffer.register_displacement(
        real=displacement_real_significant,
        apparent=displacement_apparent_significant,
    )
    # get average displacements per buffer history
    (
        avg_displ_real,
        avg_displ_apparent,
    ) = get_avg_displacement_for_significant_points(
        point_cloud_history=significant_buffer
    )

    # Registering of displacements per Object
    for segment in joined_segments:
        real_sig = segment.real_significant_points
        apparent_sig = segment.apparent_significant_points
        real_displacements = dict()
        apparent_displacements = dict()

        for point in real_sig:
            displacement: Point = avg_displ_real[point]
            real_displacements[point] = displacement

        for point in apparent_sig:
            displacement: Point = avg_displ_apparent[point]
            apparent_displacements[point] = displacement

        segment.register_displacement_real_significant_points(real_displacements)
        segment.register_displacement_apparent_significant_points(
            apparent_displacements
        )
        segment.set_average_displacement()

    center_x = list()
    center_y = list()
    velocity_x = list()
    velocity_y = list()
    speed = list()
    radius = list()
    # display of vector
    time_step = significant_buffer.get_time_between()
    for segment in joined_segments:
        shape = segment.get_shape()
        if segment.avg_displacement is None:
            continue
        if segment.avg_displacement == Point(0, 0):
            continue
        disp = segment.avg_displacement

        center_speed = segment.avg_displacement.range / time_step

        shape_patch = None
        if shape.shape_type == "circle":
            shape_patch = Circle(
                (shape.center.x, shape.center.y),
                shape.significant_length / 2,
                color="r",
                alpha=0.3,
            )
        else:
            shape_patch = Rectangle(
                xy=(shape.center.x, shape.center.y),
                width=shape.significant_length,
                height=shape.significant_length,
                angle=180 * (segment.center.angle + shape.angle) / math.pi,  # type: ignore
                color="r",
                alpha=0.3,
            )

        if shape_patch:
            ax.add_patch(shape_patch)

        if center_speed < GlobalConfig.SPEED_THRESHOLD:
            continue

        shape_center = shape_patch.get_center()

        lidar.quiver(
            *np.array([shape_center[0], shape_center[1]]),
            np.array(disp.x),
            np.array(disp.y),
            color="black",
            scale=0.9 / GlobalConfig.VECTOR_SCALING,
        )

        lidar.annotate(
            str(round(center_speed, 2)) + " m/s",
            xy=(segment.center.x, segment.center.y),
        )

        center_x.append(shape_center[0])
        center_y.append(shape_center[1])
        velocity_x.append(disp.x / time_step)
        velocity_y.append(disp.y / time_step)
        speed.append(center_speed)
        radius.append(segment.get_length() / 2)

    msg = MovingObstacles()
    msg.number_of_obstacles = len(center_x)
    msg.center_x = center_x
    msg.center_y = center_y
    msg.velocity_x = velocity_x
    msg.velocity_y = velocity_y
    msg.speed = speed
    msg.radius = radius

    time_step = (
        GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT
        * GlobalConfig.ANIMATION_INTERVAL
        / 1000
    )
    publisher.pub.publish(msg)
    et = time.time()
    frame_time = et - st
    print("total", frame_time, "s")


# Main
if __name__ == "__main__":
    # configure plot
    fig = plt.figure()
    lidar = plt.subplot()
    ax = plt.gca()

    rospy.init_node("simpleSubOOP")
    subObj = SimpleSubscriber()
    publisher = Publisher()

    significant_buffer = SignificantBuffer(size=GlobalConfig.BUFFER_SIZE)

    while not rospy.is_shutdown():
        st = time.time()
        animate(
            scan=subObj.points,
            significant_buffer=significant_buffer,
            lidar=lidar,
            publisher=publisher,
        )
        plt.pause(GlobalConfig.ANIMATION_INTERVAL / 1000)

# TODO: add proper boundaries to message
