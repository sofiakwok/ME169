#!/usr/bin/env python
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

import encoder
import driver

from sensor_msgs.msg import JointState


class WheelController:
    def __init__(self):
        # Set up low level
        self.encoder = encoder.Encoder(chLA=23, chLB=25, chRA=22, chRB=24)
        self.driver = driver.Driver(chL=0, chR=1, reverseL=0, reverseR=1)

        # Create a publisher to send the wheel desired and actual (state).
        pubdes = rospy.Publisher("/wheel_desired", JointState, queue_size=10)
        pubact = rospy.Publisher("/wheel_state", JointState, queue_size=10)

        # Create a subscriber to listen to wheel commands.
        sub = rospy.Subscriber("/wheel_command", JointState, self.callback_command)

    #
    #   Command Callback Function
    #
    #   Save the command and the time received.
    #
    def callback_command(self, msg):
        # Check the message?

        # Note the current time (to timeout the command).
        now = rospy.Time.now()

        # Save...
        # cmdvel[]  = []
        # cmdtime[] = []

    #
    #   Timer Callback Function
    #
    def callback_timer(self, event):
        # Note the current time to compute dt and populate the ROS messages.
        now = rospy.Time.now()

        # Process the commands.

        # Process the encoders, convert to wheel angles!
        self.encoder.leftencoder()
        self.encoder.rightencoder()
        # Add feedback?

        # Generate motor commands (convert wheel speed to PWM)
        # Send wheel commands.

        # Publish the actual wheel state
        msg = JointState()
        msg.header.stamp = now
        msg.name = ["leftwheel", "rightwheel"]
        msg.position = [FIRST, SECOND]
        msg.velocity = [FIRST, SECOND]
        msg.effort = [0.0, 0.0]
        pubact.publish(msg)

        # Publish the desired wheel state
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["leftwheel", "rightwheel"]
        msg.position = [FIRST, SECOND]
        msg.velocity = [FIRST, SECOND]
        msg.effort = [PWM1, PWM2]
        pubdes.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("wheelcontrol")

    # Create the timer.
    duration = rospy.Duration(DT)
    dt = duration.to_sec()
    timer = rospy.Timer(duration, callback_timer)

    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
