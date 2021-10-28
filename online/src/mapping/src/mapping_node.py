#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt

global pose
global pose_bool

pose = Pose()
pose_bool = False


def pose_callback(msg):
    global pose
    global pose_bool

    pose = msg.pose
    pose_bool = True


def mapping():

    global pose
    global pose_bool

    rospy.init_node('mapping_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # publishers and subscribers
    map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rospy.Subscriber("pose", Marker, pose_callback)


    # ros params
    r = rospy.get_param("r")
    resolution = rospy.get_param("resolution")
    width = rospy.get_param("width")
    height = rospy.get_param("height")
    frame_id = rospy.get_param("frame_id")

    # initialize ros msgs
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = frame_id
    map_msg.info.resolution = resolution
    map_msg.info.width = round(width)
    map_msg.info.height = round(height)

    rospack = rospkg.RosPack()
    filepath = rospack.get_path("mapping") + "/config/map.csv"

    map = np.genfromtxt(filepath,delimiter=',',dtype=np.int8)

    percieved_map = np.zeros(map.shape,dtype=np.int8)

    while not rospy.is_shutdown():

        if pose_bool:
            xlow = round(max(0,(pose.position.x - r)/resolution))
            xhigh = round(min(width-1,(pose.position.x + r)/resolution))
            ylow = round(max(0,(pose.position.y - r)/resolution))
            yhigh = round(min(height-1,(pose.position.y + r)/resolution))

            percieved_map[xlow:xhigh,ylow:yhigh] = map[xlow:xhigh,ylow:yhigh]
            pose_bool = False

        # publish messages
        map_msg.data = percieved_map.ravel()
        map_pub.publish(map_msg)


        rate.sleep()

if __name__ == '__main__':
    try:
        mapping()
    except rospy.ROSInterruptException:
        pass