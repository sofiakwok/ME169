#!/usr/bin/env python3
#
#   odometry.py
#
#   This is a skeleton for the implementation of the odometry node.
#
#   Node:       /odometry
#   Publish:    /odom                   sensor_msgs/JointState
#               /odometry_transform     sensor_msgs/JointState
#   Subscribe:  /wheel_state            sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#

import math
import sys
import time
import rospy
import smbus
import numpy as np

from sensor_msgs.msg import JointState

import encoder
import driver_replacement as driver
import gyro

DT = 0.01

class Odometry:
    def __init__(self, dt):
        # Set up low level
        self.i2cbus = smbus.SMBus(1)

        self.encoder = encoder.Encoder(chLA=23, chLB=25, chRA=22, chRB=24)
        self.gyro = gyro.Gyro(self.i2cbus)

        # Create a publisher to send the wheel desired and actual (state).
        self.pubodom = rospy.Publisher("/odom", JointState, queue_size=10)
        self.pubtrans = rospy.Publisher("/odometry_transform", JointState, queue_size=10)

        # Create a subscriber to listen to wheel commands.
        self.sub = rospy.Subscriber("/wheel_state", JointState, self.callbackCommand)

        self.timer = rospy.Timer(rospy.Duration(dt), self.callbackTimer)

        self.dt = dt

        self.gyro_position = 0.0
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        
    def getGyroAngularVelocity(self):
            (omega, sat) = self.gyro.read()
            if sat:
                rospy.logwarn("gyro saturation")
            return omega
        
    def getEncoderWheelAngles(self, edge_per_rot=16, gear_ratio=45):
        return (
            np.array([self.encoder.leftencoder(), self.encoder.rightencoder()])
            / (edge_per_rot * gear_ratio)
            * (2 * math.pi)
        )

    def getEncoderWheelVelocities(self, encoder_wheel_angles, dt):
        return (encoder_wheel_angles - self.old_encoder_wheel_angles) / dt

    def callbackTimer(self, event):
        # read gyro
        gyro_velocity = self.getGyroAngularVelocity()
        self.gyro_position += dt * gyro_velocity
        
        # Process the encoders, convert to wheel angles!
        encoder_positions = self.getEncoderWheelAngles()
        encoder_velocities = self.getEncoderWheelVelocities(encoder_positions, dt)
        b = encoder_velocities
        
        #calculate v and w from encoder velocity
        r = 32.9
        d = 64.45
        J = [[r/2, -r/2], [-r/(2*d), -r/(2*d)]]
        velocity_kinematics = [[sum(J*b for J,b in zip(X_row,Y_col)) for Y_col in zip(*Y)] for X_row in X]
        
        #calculate position kinematics
        delta_p = r/2*(encoder_velocities[0] - encoder_velocities[1])
        delta_theta = r/2*(-encoder_velocities[0] - encoder_velocities[1])
        
        self.theta += delta_theta
        
        delta_x = delta_p*(np.cos(self.theta + delta_theta/2))
        delta_y = delta_p*(np.sin(self.theta + delta_theta/2))
        
        self.x += delta_x
        self.y += delta_y
        
        # Publish the actual wheel state
        msg = JointState()
        #fix the time stamp heading
        msg.header.stamp = now
        msg.name = ["x", "y", "theta"]
        msg.position = np.concatenate(
            (self.x, self.y, self.theta)
        )
        #Fix velocities
        msg.velocity = np.concatenate(
            ([0.0, 0.0, 0.0])
        )
        msg.effort = [0.0, 0.0, 0.0]
        self.pubodom.publish(msg)
        
        #publish transform

    def shutdown(self):
            # Clean up the low level.
            self.encoder.shutdown()
            self.timer.shutdown()

if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("odometry")
    
    controller = Odometry(
        DT
    )
    rospy.loginfo("Running with dt = %.3f sec..." % DT)

    # Spin while the callbacks are doing all the work.
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    controller.shutdown()

