#!/usr/bin/env python
#
#   readcamconfig.py
#
#   Simply grab the camera height and angle from the TFs (this assumes
#   the robot_state_publisher is running).
#
import math
import rospy
import tf2_ros


#
#   Grab the Camera Height and Angle
#
def grabCameraConfig():
    # Create a TF buffer and listener.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Try to grab the base to camera_link transform.  Use
    # rospy.Time(0) to grab the latest transform, as opposed to the
    # transform at a particular time.  Also define a 5sec timeout, in
    # case no transforms appear.
    try:
        # Grab the transform as a TransformStamped message.
        msg = tfBuffer.lookup_transform('base', 'camera_link',
                                        rospy.Time(0), rospy.Duration(5.0))
    except Exception as ex:
        rospy.logerr("Unable to get the base to camera_link transform!")
        rospy.logerr("Have you started the robot_state_publisher?")
        raise ex

    # Remove the listener (don't need any more information).
    listener.unregister()

    # Extract the rotation.
    r = msg.transform.rotation

    # Check the camera being horizontal
    if (r.y*r.z + r.w*r.x != 0):
        errstr = "The camera is not mounted horizontally"
        rospy.logerr(errstr)
        raise rospy.ROSException(errstr)

    # Grab the angle.
    sinalpha = 2*(r.x*r.z - r.w*r.y)
    cosalpha = 1 - 2*(r.y**2 + r.z**2)
    alpha = math.atan2(sinalpha, cosalpha)

    # Also grab the height.
    height = msg.transform.translation.z

    # Return the data.
    return((height, alpha))


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('readcamconfig')

    # Grab the camera height and angle.
    (height, alpha) = grabCameraConfig()
    rospy.loginfo("The camera is located %3.2f mm above ground" %
                  (height*1000))
    rospy.loginfo("The camera is tilted up by %3.1f deg = %4.3f rad" %
                  (alpha * 180/math.pi, alpha))
