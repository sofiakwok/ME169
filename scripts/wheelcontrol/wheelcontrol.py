#!/usr/bin/env python3
#
#   wheelcontrol_skeleton.py
#
#   This is a skelenton for the implementation of the Wheel-Control node.  Beyond
#   the basic, it should
#     - stops if no commands are received after 0.25sec
#     - filter the commands to avoid spikes (high torque/current loads)
#     - filters the actual velocity
#     - adds feedback to perfectly achieve the velocity commands
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
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
ENCODER_FILTER_T = 0.2
DESIRED_FILTER_T = 0.4
POSITION_CORRECTIVE_T = 1


class WheelController:
    def __init__(self, dt, eft, dft, pct):
        # Set up low level
        self.i2cbus = smbus.SMBus(1)

        self.encoder = encoder.Encoder(chLA=23, chLB=25, chRA=22, chRB=24)
        self.driver = driver.Driver(self.i2cbus, chL=0, chR=1, reverseL=0, reverseR=0)
        self.gyro = gyro.Gyro(self.i2cbus)

        self.dt = dt
        self.encoder_filter_t = eft
        self.encoder_filter_velocities = np.array([0.0, 0.0])

        self.old_encoder_wheel_angles = self.getEncoderWheelAngles()
        self.old_encoder_now = rospy.Time.now()

        self.cmd_velocities = np.array([0.0, 0.0])
        self.cmd_time = rospy.Time.now()

        self.desired_filter_t = dft
        self.old_desired_positions = np.array([0.0, 0.0])
        self.desired_filter_velocities = np.array([0.0, 0.0])

        self.position_corrective_t = pct

        self.gyro_position = 0.0

        # Create a publisher to send the wheel desired and actual (state).
        self.pubdes = rospy.Publisher("/wheel_desired", JointState, queue_size=10)
        self.pubact = rospy.Publisher("/wheel_state", JointState, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(dt), self.callbackTimer)

        # Create a subscriber to listen to wheel commands.
        self.sub = rospy.Subscriber("/wheel_command", JointState, self.callbackCommand)

    #
    #   Command Callback Function
    #
    #   Save the command and the time received.
    #
    def callbackCommand(self, msg, time_thresh=0.25):
        # Check the message?

        # Note the current time (to timeout the command).
        now = rospy.Time.now()

        if (now - msg.header.stamp).to_sec() < time_thresh:
            self.old_desired_positions += (
                self.cmd_velocities * (now - self.cmd_time).to_sec()
            )

            self.cmd_velocities = np.array(msg.velocity)
            self.cmd_time = msg.header.stamp

    def getEncoderWheelAngles(self, edge_per_rot=16, gear_ratio=45):
        return (
            np.array([self.encoder.leftencoder(), self.encoder.rightencoder()])
            / (edge_per_rot * gear_ratio)
            * (2 * math.pi)
        )

    def getEncoderWheelVelocities(self, encoder_wheel_angles, dt):
        return (encoder_wheel_angles - self.old_encoder_wheel_angles) / dt

    def getGyroAngularVelocity(self):
        (omega, sat) = self.gyro.read()
        if sat:
            rospy.logwarn("gyro saturation")
        return omega

    def applyFilter(self, u, x, dt, filter_t):
        return u + (1 / filter_t) * dt * (x - u)

    def getDt(self, now):
        dt = now - self.old_encoder_now
        self.old_encoder_now = now
        return dt.to_sec()

    def velocityToPWM(self, velocity):
        return ((velocity > 0) * 35 + (12 * velocity)) + (
            (velocity < 0) * (-35 + 12 * velocity)
        )

    #
    #   Timer Callback Function
    #

    def callbackTimer(self, event):
        now = rospy.Time.now()
        # Note the current time to compute dt and populate the ROS messages.
        dt = self.getDt(now)

        # Process the commands.
        self.desired_filter_velocities = self.applyFilter(
            self.desired_filter_velocities,
            self.cmd_velocities,
            dt,
            self.desired_filter_t,
        )
        desired_positions = (
            self.old_desired_positions
            + self.cmd_velocities * (now - self.cmd_time).to_sec()
        )

        # Process the encoders, convert to wheel angles!
        encoder_positions = self.getEncoderWheelAngles()
        encoder_velocities = self.getEncoderWheelVelocities(encoder_positions, dt)
        self.encoder_filter_velocities = self.applyFilter(
            self.encoder_filter_velocities,
            encoder_velocities,
            dt,
            self.encoder_filter_t,
        )

        self.old_encoder_wheel_angles = encoder_positions

        # read gyro
        gyro_velocity = self.getGyroAngularVelocity()
        self.gyro_position += dt * gyro_velocity

        # Generate motor commands (convert wheel speed to PWM)
        # Send wheel commands.
        desired_efforts = self.velocityToPWM(
            self.desired_filter_velocities
            + (1 / self.position_corrective_t) * (desired_positions - encoder_positions)
        )
        self.driver.left(desired_efforts[0])
        self.driver.right(desired_efforts[1])

        # Publish the actual wheel state
        msg = JointState()
        msg.header.stamp = now
        msg.name = ["leftwheel", "rightwheel", "gyro"]
        msg.position = np.concatenate(
            (encoder_positions, np.array([self.gyro_position]))
        )
        msg.velocity = np.concatenate(
            (self.encoder_filter_velocities, np.array([gyro_velocity]))
        )
        msg.effort = [0.0, 0.0, 0.0]
        self.pubact.publish(msg)

        # Publish the desired wheel state
        msg = JointState()
        msg.header.stamp = now
        msg.name = ["leftwheel", "rightwheel"]
        msg.position = desired_positions
        msg.velocity = self.desired_filter_velocities
        msg.effort = desired_efforts
        self.pubdes.publish(msg)

    def shutdown(self):
        # Clean up the low level.
        self.driver.shutdown()
        self.encoder.shutdown()
        self.timer.shutdown()


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("wheelcontrol")

    controller = WheelController(
        DT, ENCODER_FILTER_T, DESIRED_FILTER_T, POSITION_CORRECTIVE_T
    )
    rospy.loginfo("Running with dt = %.3f sec..." % DT)

    # Spin while the callbacks are doing all the work.
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    controller.shutdown()
