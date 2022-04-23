#!/usr/bin/env python3
#
#   odometry.py
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import rospy
import tf2_ros

import numpy as np

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


#
#   Constants
#
R = 0.045  # 0.0329  # Wheel radius = 32.9 mm
d = 0.089  # 0.06445  # Halfwidth between wheels = 64.45 mm


#
#   Odometry Object
#
class OdometryObj:
    # Initialize.
    def __init__(self):
        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.J = np.array([[R / 2, R / 2], [-R / (2 * d), R / (2 * d)]])
        self.Ji = np.linalg.inv(self.J)

        # Create a publisher to send wheel commands.
        self.pub_wcmd = rospy.Publisher("/wheel_command", JointState, queue_size=3)

        # Create a publisher to send odometry information.
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Create a TF2 transform broadcaster.
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to twist commands.
        rospy.Subscriber("/vel_cmd", Twist, self.cb_vel_cmd)

        # Create a subscriber to listen to wheel state.
        rospy.Subscriber("/wheel_state", JointState, self.cb_wheel_state)

        self.old_wheel_state_now = rospy.Time.now()

    # Velocity Command Message Callback
    def cb_vel_cmd(self, msg):
        psi_dot = self.Ji @ np.array([msg.linear.x, msg.angular.z]).reshape((2, 1))

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["leftwheel", "rightwheel"]
        msg.velocity = [psi_dot[0], psi_dot[1]]
        self.pub_wcmd.publish(msg)

    def getDt(self, now):
        dt = now - self.old_wheel_state_now
        self.old_wheel_state_now = now
        return dt.to_sec()

    # Wheel State Message Callback
    def cb_wheel_state(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        now = msg.header.stamp
        dt = self.getDt(now)

        psi_dot = np.array(
            [
                msg.velocity[msg.name.index("leftwheel")],
                msg.velocity[msg.name.index("rightwheel")],
            ]
        )

        gyro_theta_dot = msg.velocity[msg.name.index("gyro")]

        vx = (
            R / 2 * psi_dot[1] - (gyro_theta_dot - R / (2 * d) * psi_dot[1]) * d
            if abs(psi_dot[0]) > abs(psi_dot[1])
            else R / 2 * psi_dot[0] + (gyro_theta_dot + R / (2 * d) * psi_dot[0]) * d
        )

        wz = gyro_theta_dot

        # Update the pose.
        if abs(wz) < 0.0001:
            self.x += dt * vx * math.cos(self.theta)
            self.y += dt * vx * math.sin(self.theta)
        else:
            self.x += (
                dt
                * vx
                * math.cos(self.theta + dt * wz / 2)
                * math.sin(dt * wz / 2)
                / (dt * wz / 2)
            )
            self.y += (
                dt
                * vx
                * math.sin(self.theta + dt * wz / 2)
                * math.sin(dt * wz / 2)
                / (dt * wz / 2)
            )
        self.theta += dt * wz

        # Convert to a ROS Point, Quaternion, Twist (lin&ang veloocity).
        p = Point(self.x, self.y, 0.0)
        q = Quaternion(0.0, 0.0, math.sin(self.theta / 2), math.cos(self.theta / 2))
        t = Twist(Vector3(vx, 0.0, 0.0), Vector3(0.0, 0.0, wz))

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base"
        msg.pose.pose.position = p
        msg.pose.pose.orientation = q
        msg.twist.twist = t
        self.pub_odom.publish(msg)

        # Create the transform msg and broadcast (reuse the time stamp).
        msg = TransformStamped()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base"
        msg.transform.translation = p
        msg.transform.rotation = q
        self.brd_tf.sendTransform(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("odometry")

    # Instantiate the Odometry object
    odometry = OdometryObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Odometry spinning...")
    rospy.spin()
    rospy.loginfo("Odometry stopped.")
