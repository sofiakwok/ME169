#!/usr/bin/env python3
#
#   localization.py
#
#   Localize by adjusting map to odom transform
#
#   Node:       /localization
#   Publish:    /pose             geometry_msgs/PoseStamped
#   Subscribe:  /odom             nav_msgs/Odometry
#               /scan             LaserScan
#               /initialpose      geometry_msgs/PoseStamped
import rospy
import math
import numpy as np
import tf2_ros

from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    TransformStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from PlanarTransform import PlanarTransform


class Localization:
    def __init__(self) -> None:
        # Wait 30sec for a map.
        # rospy.loginfo("Waiting for a map...")
        # mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

        # self.map = mapmsg.data
        # self.map_resolution = mapmsg.info.resolution
        # self.map_origin = mapmsg.info.origin

        rospy.loginfo("Loaded map")

        self.map_to_odom = PlanarTransform.unity()

        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.brd_tf = tf2_ros.TransformBroadcaster()

        tf_buffer = tf2_ros.Buffer()
        self.lis_tf = tf2_ros.TransformListener(tf_buffer)

        rospy.Subscriber("/odom", Odometry, self.updateOdometry)
        rospy.Subscriber("/scan", LaserScan, self.updateScan)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.updatePose)

    def updateOdometry(self, odom_msg):
        odom_to_base = PlanarTransform.fromPose(odom_msg.pose.pose)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_msg.header.stamp
        pose_msg.pose = (self.map_to_odom * odom_to_base).toPose()
        self.pose_pub.publish(pose_msg)

    def updateScan(self, msg):
        self.broadcastMapToOdom(msg.header.stamp)

    def updatePose(self, msg):
        self.map_to_odom = PlanarTransform.fromPose(msg.pose.pose)
        self.broadcastMapToOdom(msg.header.stamp)

    def broadcastMapToOdom(self, time):
        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        msg.transform = self.map_to_odom.toTransform()
        self.brd_tf.sendTransform(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("localization")

    localization = Localization()

    rospy.loginfo("localization started")
    rospy.spin()
    rospy.loginfo("localization stopped.")
