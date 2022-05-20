#!/usr/bin/env python3
#
#   simpledriver.py
#
#   Continually (at 100Hz!) naviage to 2D navgoal position.
#
#   Node:       /simpledriver
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  /pose             geometry_msgs/PoseStamped
#               /move_base_simple/goal  geometry_msgs/PoseStamped
import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import rrt, map
from PlanarTransform import PlanarTransform

MIN_OBSTACLE_DIST = 0.3
MAX_TARGET_DIST = 0.1
MAX_TARGET_THETA = 0.1

MAX_SPEED = 0.1
DIST_SCALE_SPEED = 0.3

MAX_TURN = 0.5
TURN_SCALE = 0.5

CONF_THRESHOLD = 0.75


class Waypointdriver:
    def __init__(self) -> None:
        self.localization_conf = 0

        self.cx = 0.0
        self.cy = 0.0
        self.ctheta = 0.0

        self.waypoints = []
        self.ttheta = None

        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        self.closestObstacle = 0.0

        rospy.loginfo("Loaded map")

        self.pub = rospy.Publisher("/vel_cmd", Twist, queue_size=10)
        self.pointspub = rospy.Publisher("/waypoints", Marker, queue_size=10)
        rospy.Subscriber("/pose", PoseStamped, self.updatePose)
        rospy.Subscriber("/scan", LaserScan, self.updateDistance)

        self.servo = rospy.Rate(100)
        self.dt = self.servo.sleep_dur.to_sec()

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
        self.rrt = rrt.RRT(self.map)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateTarget)
        rospy.Subscriber("/localization_conf", Float32, self.updateLocalizationConf)

    def updateWaypoints(self):
        marker_pts = Marker()
        marker_pts.header.frame_id = "map"
        marker_pts.header.stamp = rospy.Time.now()
        marker_pts.ns = "waypoints"
        marker_pts.id = 0
        marker_pts.type = Marker.LINE_STRIP
        marker_pts.action = Marker.ADD
        marker_pts.scale.x = 0.05
        marker_pts.scale.y = 0.05
        marker_pts.scale.z = 0.05
        marker_pts.color.a = 1.0
        marker_pts.color.r = 0.0
        marker_pts.color.g = 1.0
        marker_pts.color.b = 0.0

        marker_pts.points = []

        marker_pts.points.append(Point(self.cx, self.cy, 0.0))
        for pt in self.waypoints:
            marker_pts.points.append(Point(pt[0], pt[1], 0.0))
        self.pointspub.publish(marker_pts)

    def updateLocalizationConf(self, conf_msg):
        self.localization_conf = conf_msg.data

    def updateDistance(self, msg):
        ranges = np.array(msg.ranges)
        second_smallest = np.percentile(ranges[np.nonzero(ranges)], 5)
        self.closestObstacle = second_smallest

    def updatePose(self, msg):
        self.cx = msg.pose.position.x
        self.cy = msg.pose.position.y
        self.ctheta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def updateTarget(self, msg):
        # self.waypoints.append([msg.pose.position.x, msg.pose.position.y])
        self.waypoints = self.rrt.pathRRT(
            [self.cx, self.cy], [msg.pose.position.x, msg.pose.position.y]
        )
        # self.updateWaypoints()
        self.ttheta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def loop(self):
        # Run the servo loop until shutdown.
        while not rospy.is_shutdown():
            if len(self.waypoints) > 0 and self.localization_conf > CONF_THRESHOLD:
                tx = self.waypoints[0][0]
                ty = self.waypoints[0][1]
                dist = math.sqrt((tx - self.cx) ** 2 + (ty - self.cy) ** 2)

                if dist < MAX_TARGET_DIST:
                    self.waypoints.pop(0)
                    # self.updateWaypoints()

                else:
                    gtheta = math.atan2(ty - self.cy, tx - self.cx)

                dtheta = gtheta - self.ctheta
                dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

                if dist < MAX_TARGET_DIST or self.closestObstacle < MIN_OBSTACLE_DIST:
                    self.msg.linear.x = 0
                else:
                    self.msg.linear.x = max(
                        min(
                            MAX_SPEED, DIST_SCALE_SPEED * dist * (math.cos(dtheta) ** 5)
                        ),
                        -MAX_SPEED,
                    )

                self.msg.angular.z = min(max(TURN_SCALE * dtheta, -MAX_TURN), MAX_TURN)
            else:
                self.msg.linear.x = 0
                self.msg.angular.z = 0

            self.pub.publish(self.msg)
            self.updateWaypoints()

            # Wait for the next turn.
            self.servo.sleep()


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("simpledriver")

    simpledriver = Waypointdriver()

    rospy.loginfo("simpledriver started")
    simpledriver.loop()

    # Report the exit.
    rospy.loginfo("Stopping transmissions.")
