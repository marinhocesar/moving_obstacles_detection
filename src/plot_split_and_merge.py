import time
from typing import List
import ydlidar
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from aux.iterative_closest_point import (
    get_avg_displacement_for_significant_points,
    get_displacement_for_significant_points,
)
from components.buffer import SignificantBuffer

from components.point import Point
from components.object import Object
from aux.global_config import GlobalConfig
from aux import split_and_merge, filtering, point_extractor


# configure plot
fig = plt.figure()
lidar = plt.subplot()
ax = plt.gca()

# configure LiDAR
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
scan = ydlidar.LaserScan()


def animate(num, previous_points: dict, significant_buffer: SignificantBuffer):
    # st = time.time()
    # last = time.time()
    r = laser.doProcessSimple(scan)
    if r:
        # configure plot
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
        structured_data = [
            Point(range=point.range, angle=-point.angle) for point in scan.points
        ]
        filtered_data = filtering.filter_data(data=structured_data)
        segments = split_and_merge.segment_lidar_data(
            scan_points=filtered_data, previous_points=previous_points
        )
        previous_points = filtering.get_points_from_segments(
            segments=segments, dict=previous_points
        )
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
                lidar.scatter(
                    real_significant.x_pos, real_significant.y_pos, color="red"
                )

            if segment.apparent_significant_points:
                apparent_significant = Object(
                    points=segment.apparent_significant_points
                )
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
            time=time.time() - st,
        )

        # get displacements
        (
            displacement_real_significant,
            displacement_apparent_significant,
        ) = get_displacement_for_significant_points(
            point_cloud_history=significant_buffer
        )
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

        # display of vector
        for segment in joined_segments:
            if segment.avg_displacement is None:
                continue
            if segment.avg_displacement == Point(0, 0):
                continue

            # segment.set_velocity_vector_for_display()
            # x_vel = segment.velocity_x
            # y_vel = segment.velocity_y
            # lidar.plot(x_vel, y_vel, "-", color="black")

            disp = segment.avg_displacement

            # time step in seconds
            time_step = GlobalConfig.DIFFERENCE_FOR_DISPLACEMENT * GlobalConfig.ANIMATION_INTERVAL / 1000

            speed = segment.avg_displacement.range / time_step

            # if speed < GlobalConfig.SPEED_THRESHOLD:
            #     continue

            lidar.quiver(
                *np.array([segment.center.x, segment.center.y]),
                np.array(disp.x),
                np.array(disp.y),
                color="black",
                scale=0.9 / GlobalConfig.VECTOR_SCALING,
            )

            lidar.annotate(
                str(round(speed, 2)) + " m/s",
                xy=(segment.center.x, segment.center.y),
            )
        # et = time.time()
        # frame_time = et - st
        # print(frame_time, " s")


ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    previous_points = dict()  # dict that stores previous_points[angle] = range
    significant_buffer = SignificantBuffer(size=GlobalConfig.BUFFER_SIZE)
    st = time.time()
    last = st
    if ret:
        ani = animation.FuncAnimation(
            fig,
            animate,
            interval=GlobalConfig.ANIMATION_INTERVAL,
            fargs=(previous_points, significant_buffer),
        )
        plt.show()
    laser.turnOff()
    get_displacement_for_significant_points(significant_buffer)

    # print(f"index: {significant_buffer.last_record % significant_buffer.size}")
    # print([len(lista) for lista in significant_buffer.real_significant])
    # print([len(lista) for lista in significant_buffer.apparent_significant])

laser.disconnecting()
plt.close()


# TODO: estimate velocity (direction) for moving objects
