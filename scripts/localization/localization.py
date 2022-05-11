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

WEIGHT = 0.1


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

        self.map_to_odom = PlanarTransform.unity()
        self.odom_to_base = PlanarTransform.unity()

        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.brd_tf = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.lis_tf = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/odom", Odometry, self.updateOdometry)
        rospy.Subscriber("/scan", LaserScan, self.updateScan)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.updatePose)

    def updateOdometry(self, odom_msg):
        self.odom_to_base = PlanarTransform.fromPose(odom_msg.pose.pose)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_msg.header.stamp
        pose_msg.pose = (self.map_to_odom * self.odom_to_base).toPose()
        self.pose_pub.publish(pose_msg)

    def updateScan(self, msg):
        tfmsg = self.tf_buffer.lookup_transform(
            "odom", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)
        )
        map_to_laser = self.map_to_odom * PlanarTransform.fromTransform(tfmsg.transform)

        scan_pts, map_pts = np.array(
            self.map.nearestWallptsFromScan(
                msg.ranges,
                msg.angle_min,
                msg.angle_max,
                msg.range_min,
                msg.range_max,
                map_to_laser,
            )
        )

        if len(scan_pts) > 10:
            a = np.linalg.norm(map_pts - scan_pts, axis=1)

            J = (
                np.array(
                    [
                        map_pts[:, 0] - scan_pts[:, 0],
                        map_pts[:, 1] - scan_pts[:, 1],
                        map_pts[:, 1] * scan_pts[:, 0] - map_pts[:, 0] * scan_pts[:, 1],
                    ]
                ).T
                / a[:, None]
            )

            # base_to_map = (self.map_to_odom * self.odom_to_base).inv()

            # scan_weights = np.minimum(
            #     1 / np.linalg.norm(base_to_map.inParentArray(scan_pts), axis=1),
            #     1,
            # )
            # w = np.diag(scan_weights)

            # delta = np.linalg.pinv(J.T @ w @ J) @ J.T @ w @ a
            delta = np.linalg.pinv(J.T @ J) @ J.T @ a

            original_x = self.map_to_odom.x()
            original_y = self.map_to_odom.y()
            original_theta = self.map_to_odom.theta()

            delta_w = WEIGHT * delta  # * (np.mean(scan_weights))

            self.map_to_odom = PlanarTransform.basic(
                delta_w[0] + original_x,
                delta_w[1] + original_y,
                delta_w[2] + original_theta,
            )

        self.broadcastMapToOdom(msg.header.stamp)

    def updatePose(self, msg):
        map_to_base = PlanarTransform.fromPose(msg.pose.pose)
        self.map_to_odom = map_to_base * self.odom_to_base.inv()

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
