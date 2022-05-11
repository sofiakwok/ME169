#!/usr/bin/env python3
#
#   depthonline.py
#
#   Report the depth on the horizontal center line of the image.
#
import numpy as np
import rospy
import math

from sensor_msgs.msg import Image


#
#   Depth on Center-Line of Image
#
def depthCB(msg):
    # Extract the depth image information (distance in mm as uint16).
    width = msg.width
    height = msg.height
    depth = np.frombuffer(msg.data, np.uint16).reshape(height, width)

    # Report.
    col = int(width / 2)
    row = int(height / 2)

    # rospy.loginfo("Distance at (row %d, col %d) = %dmm" % (row, col, d))

    xs = []
    ys = []
    fy_inv = 1 / 425.9604187011719
    for r in range(0, height):

        z = depth[r, col]
        y = z * ((r - row) * fy_inv)
        if z != 0:
            xs.append(z)
            ys.append(y)

    A = np.vstack([np.array(xs), np.ones(len(xs))]).T
    m, c = np.linalg.lstsq(A, np.array(ys), rcond=None)[0]

    print(math.pi / 2 + math.atan(m), c)
    # rospy.loginfo("Line = %s" % str(line))


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node("depthonline")

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, depthCB)

    # Report, then continually process until shutdown.
    rospy.loginfo("Starting the depth-on-center-line tracking...")
    rospy.spin()
    rospy.loginfo("Done!")
