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

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

MIN_OBSTACLE_DIST = 0.3
MAX_TARGET_DIST = 0.05
MAX_SPEED = 0.2
MAX_TURN = 2


class Simpledriver:
    def __init__(self) -> None:
        self.cx = 0.0
        self.cy = 0.0
        self.ctheta = 0.0

        self.tx = 0.0
        self.ty = 0.0
        self.ttheta = 0.0

        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        self.closestObstacle = 0.0

        self.pub = rospy.Publisher("/vel_cmd", Twist, queue_size=10)
        rospy.Subscriber("/pose", PoseStamped, self.updatePose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateTarget)
        rospy.Subscriber("/scan", LaserScan, self.updateDistance)

        self.servo = rospy.Rate(100)
        self.dt = self.servo.sleep_dur.to_sec()

    def updateDistance(self, msg):
        ranges = np.array(msg.ranges)
        second_smallest = np.percentile(ranges[np.nonzero(ranges)], 5)
        self.closestObstacle = second_smallest

    def updatePose(self, msg):
        self.cx = msg.pose.position.x
        self.cy = msg.pose.position.y
        self.ctheta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def updateTarget(self, msg):
        self.tx = msg.pose.position.x
        self.ty = msg.pose.position.y
        self.ttheta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def loop(self):
        # Run the servo loop until shutdown.
        while not rospy.is_shutdown():
            dist = math.sqrt((self.tx - self.cx) ** 2 + (self.ty - self.cy) ** 2)

            if dist < MAX_TARGET_DIST:
                gtheta = self.ttheta
            else:
                gtheta = math.atan2(self.ty - self.cy, self.tx - self.cx)

            dtheta = gtheta - self.ctheta
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

            if dist < MAX_TARGET_DIST or self.closestObstacle < MIN_OBSTACLE_DIST:
                self.msg.linear.x = 0
            else:
                self.msg.linear.x = max(
                    min(MAX_SPEED, 0.5 * dist * math.cos(dtheta)), -MAX_SPEED
                )

            self.msg.angular.z = min(max(dtheta, -MAX_TURN), MAX_TURN)

            self.pub.publish(self.msg)

            # Wait for the next turn.
            self.servo.sleep()


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("simpledriver")

    simpledriver = Simpledriver()

    rospy.loginfo("simpledriver started")
    simpledriver.loop()

    # Report the exit.
    rospy.loginfo("Stopping transmissions.")
