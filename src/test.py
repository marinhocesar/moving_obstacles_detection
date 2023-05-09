import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class mySub:
    def __init__(self):
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)
        self.laser_scan = LaserScan()
        self.angles = list()

    def callback(self, msg: LaserScan):
        self.laser_scan = msg
        self.angles = [msg.angle_min + i for i in np.arange(0, msg.angle_max, msg.angle_increment)]

    def printMsg(self):
        print(self.angles)


# Main
if __name__ == "__main__":
    rospy.init_node("simpleSubOOP")
    subObj = mySub()

    while not rospy.is_shutdown():
        subObj.printMsg()
