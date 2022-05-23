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
import map as worldmap
import random
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

WEIGHT = 0.05
MAX_UPDATE = 0.01
NMCFRAMES = 20
MAXPTS = 40
RANDOMIZE_THRESH = 0.4
SWITCH_DELTA_THRESH = 0.05
SWITCH_THESH = 0.7

PREV_CONF_BUFFER = 0.05

MIN_DIST = 0.1
MIN_ANGLE = 0.1


class MonteCarloLocalization:
    def __init__(self) -> None:

        self.localization_conf = 0
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

        rospy.loginfo("Finding nearest walls...")
        init_map = np.array(mapmsg.data).reshape(
            (mapmsg.info.height, mapmsg.info.width)
        )
        self.map = worldmap.Map(
            init_map,
            mapmsg.info.resolution,
            PlanarTransform.fromPose(mapmsg.info.origin),
        )

        rospy.loginfo("Loaded map")

        self.brd_tf = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.lis_tf = tf2_ros.TransformListener(self.tf_buffer)

        # set up frames
        self.base_to_laser = PlanarTransform.fromTransform(
            self.tf_buffer.lookup_transform(
                "base", "laser", rospy.Time.now(), rospy.Duration(1.0)
            ).transform
        )

        self.odom_to_base = PlanarTransform.unity()
        self.map_to_odom_mcframes = []
        for i in range(NMCFRAMES):
            f = MonteCarloFrame(
                "odom" + str(i),
                self.brd_tf,
                self.tf_buffer,
                self.map,
                self.base_to_laser,
            )
            f.randomize()
            self.map_to_odom_mcframes.append(f)

        self.switch_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        self.conf_pub = rospy.Publisher("/mclocalization_conf", Float32, queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.updateOdometry)
        rospy.Subscriber("/scan", LaserScan, self.updateScan)
        rospy.Subscriber("/localization_conf", Float32, self.updateLocalizationConf)

    def updateLocalizationConf(self, conf_msg):
        self.localization_conf = conf_msg.data

    def updateOdometry(self, odom_msg):
        self.odom_to_base = PlanarTransform.fromPose(odom_msg.pose.pose)

    def updateScan(self, msg):
        odom_to_base_msg = self.tf_buffer.lookup_transform(
            "odom", "base", msg.header.stamp, rospy.Duration(0.1)
        )

        odom_to_base = PlanarTransform.fromTransform(odom_to_base_msg.transform)
        laser_frame_scan_locs = self.map.filterScan(
            msg.ranges,
            msg.angle_min,
            msg.angle_max,
            msg.range_min,
            msg.range_max,
            max_pts=MAXPTS,
        )

        known_locs = []
        for f in self.map_to_odom_mcframes:
            fl = f.lookupRecent()
            for l in known_locs:
                if l.dist(fl) < MIN_DIST:  # and abs(l.angledist(fl)) < MIN_ANGLE:
                    f.randomize()
                else:
                    known_locs.append(fl)

        for f in self.map_to_odom_mcframes:
            f.localize(laser_frame_scan_locs, odom_to_base, msg.header.stamp, WEIGHT)

            if (
                f.conf < RANDOMIZE_THRESH
                or f.conf < f.prev_confs[0] < f.conf + PREV_CONF_BUFFER
            ):
                f.randomize()

        self.map_to_odom_mcframes.sort(key=lambda f: -f.conf)
        if (
            self.map_to_odom_mcframes[0].conf
            > (self.localization_conf + SWITCH_DELTA_THRESH)
            and self.map_to_odom_mcframes[0].conf > SWITCH_THESH
        ):
            switch_msg = PoseWithCovarianceStamped()
            switch_msg.pose.pose = (
                self.map_to_odom_mcframes[0].lookupRecent() * odom_to_base
            ).toPose()
            switch_msg.header.stamp = msg.header.stamp
            self.switch_pub.publish(switch_msg)
            f.randomize()
        self.conf_pub.publish(self.map_to_odom_mcframes[0].conf)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.

    rospy.init_node("mclocalization")

    mclocalization = MonteCarloLocalization()

    rospy.loginfo("mclocalization started")
    rospy.spin()
    rospy.loginfo("mclocalization stopped.")
