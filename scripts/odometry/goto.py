#!/usr/bin/env python3
#
#   teleop.py
#
#   Continually (at 10Hz!) send the a velocity command.
#
#   Node:       /teleop
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  -none-
#
import curses
import sys
import rospy
import math

from geometry_msgs.msg import Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry


cx = 0.0
cy = 0.0
ctheta = 0.0

tx = 0.0
ty = 0.0
ttheta = 0.0

#
#   Terminal Input Loop
#


def loop():
    # Run the servo loop until shutdown.
    while not rospy.is_shutdown():
        dist = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)
        if dist < 0.05:
            gtheta = ttheta
            msg.linear.x = 0
        else:
            gtheta = math.atan2(ty - cy, tx - cx)
            msg.linear.x = min(0.2, dist)

        dtheta = gtheta - ctheta
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

        msg.angular.z = min(max(dtheta, -2), 2)

        pub.publish(msg)

        # Wait for the next turn.
        servo.sleep()


def updateOdometry(msg):
    global cx, cy, ctheta, cnow
    cx = msg.pose.pose.position.x
    cy = msg.pose.pose.position.y
    ctheta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)


def updateTarget(msg):
    global tx, ty, ttheta
    tx = msg.pose.position.x
    ty = msg.pose.position.y
    ttheta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("goto")

    # Create a publisher to send twist commands.
    pub = rospy.Publisher("/vel_cmd", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, updateOdometry)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, updateTarget)

    # Initialize the (repeating) message data.
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    # Create a servo loop at 10Hz.
    servo = rospy.Rate(100)
    dt = servo.sleep_dur.to_sec()

    # Report.
    rospy.loginfo("goto started")

    loop()

    # Report the exit.
    rospy.loginfo("Stopping transmissions.")
