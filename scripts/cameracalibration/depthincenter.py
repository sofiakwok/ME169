#!/usr/bin/env python
#
#   depthincenter.py
#
#   Report the depth in the center of the image.
#
import numpy as np
import rospy

from sensor_msgs.msg import Image


#
#   Depth in Center of Image
#
def depthCB(msg):
    # Extract the depth image information (distance in mm as uint16).
    width  = msg.width
    height = msg.height
    depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

    # Report.
    col = width/2
    row = height/2
    d   = depth[row][col]
    
    rospy.loginfo("Distance at (row %d, col %d) = %dmm" % (row, col, d))


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('depthincenter')

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depthCB)

    # Report, then continually process until shutdown.
    rospy.loginfo("Starting the depth-in-center-of-image tracking...")
    rospy.spin()
    rospy.loginfo("Done!")
