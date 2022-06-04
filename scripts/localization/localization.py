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
import map
import time

from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    TransformStamped,
    PoseWithCovarianceStamped,
    Point,
)

from visualization_msgs.msg import Marker

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from PlanarTransform import PlanarTransform
from montecarloframe import MonteCarloFrame

WEIGHT = 0.01
MAX_UPDATE = 0.01


class Localization:
    def __init__(self) -> None:
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

        rospy.loginfo("Finding nearest walls...")
        init_map = np.array(mapmsg.data).reshape(
            (mapmsg.info.height, mapmsg.info.width)
        )
        self.map = map.Map(
            init_map,
            mapmsg.info.resolution,
            PlanarTransform.fromPose(mapmsg.info.origin),
        )

        rospy.loginfo("Loaded map")

        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)
        self.alternate_pose_pub = rospy.Publisher("/mcposes", Marker, queue_size=1)
        self.conf_pub = rospy.Publisher("/localization_conf", Float32, queue_size=1)
        self.brd_tf = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.lis_tf = tf2_ros.TransformListener(self.tf_buffer)

        # set up frames
        self.base_to_laser = PlanarTransform.fromTransform(
            self.tf_buffer.lookup_transform(
                "base", "laser", rospy.Time.now(), rospy.Duration(1.0)
            ).transform
        )
        self.map_to_odom_mcframe = MonteCarloFrame(
            "odom", self.brd_tf, self.tf_buffer, self.map, self.base_to_laser
        )
        self.odom_to_base = PlanarTransform.unity()

        rospy.Subscriber("/odom", Odometry, self.updateOdometry)
        rospy.Subscriber("/scan", LaserScan, self.updateScan)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.updatePose)

    def updateOdometry(self, odom_msg):
        self.odom_to_base = PlanarTransform.fromPose(odom_msg.pose.pose)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_msg.header.stamp
        pose_msg.pose = (
            self.map_to_odom_mcframe.lookupRecent() * self.odom_to_base
        ).toPose()
        self.pose_pub.publish(pose_msg)

    def updateScan(self, msg):
        odom_to_base_msg = self.tf_buffer.lookup_transform(
            "odom", "base", msg.header.stamp, rospy.Duration(0.1)
        )

        odom_to_base = PlanarTransform.fromTransform(odom_to_base_msg.transform)
        laser_frame_scan_locs = self.map.filterScan(
            msg.ranges, msg.angle_min, msg.angle_max, msg.range_min, msg.range_max
        )
        self.map_to_odom_mcframe.localize(
            laser_frame_scan_locs, odom_to_base, msg.header.stamp, WEIGHT
        )

        conf_msg = Float32(data=self.map_to_odom_mcframe.conf)
        self.conf_pub.publish(conf_msg)

        # for f in self.map_to_odom_mcframes:
        #     f.localize(
        #         laser_frame_scan_locs, odom_to_base, msg.header.stamp, 10 * WEIGHT
        #     )

    def updatePose(self, msg):
        map_to_base = PlanarTransform.fromPose(msg.pose.pose)
        self.map_to_odom_mcframe.set(map_to_base * self.odom_to_base.inv())
        self.map_to_odom_mcframe.broadcast(msg.header.stamp)


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
