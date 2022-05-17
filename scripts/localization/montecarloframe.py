from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    TransformStamped,
    PoseWithCovarianceStamped,
)

from PlanarTransform import PlanarTransform

import rospy
import random
import numpy as np


class MonteCarloFrame:
    def __init__(
        self,
        name,
        brd_tf,
        tf_buffer,
        map,
        base_to_laser,
        tf=PlanarTransform.unity(),
        parent="map",
    ) -> None:
        self.name = name
        self.parent = parent

        self.brd_tf = brd_tf
        self.tf_buffer = tf_buffer
        self.map = map
        self.base_to_laser = base_to_laser

        self.tf = tf

        self.logconf = 1

    def randomize(self):
        self.tf = PlanarTransform.basic(
            random.normalvariate(0, 1),
            random.normalvariate(0, 1),
            random.uniform(0, 2 * np.pi),
        )

    def set(self, ntf, time=None):
        self.tf = ntf
        self.broadcast(rospy.Time.now() if time is None else time)

    def broadcast(self, time):
        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.parent
        msg.child_frame_id = self.name
        msg.transform = self.tf.toTransform()
        self.brd_tf.sendTransform(msg)

    def lookupTime(self, time):
        msg = self.tf_buffer.lookup_transform("odom", "base", time, rospy.Duration(0.1))

        return PlanarTransform.fromTransform(msg.transform)

    def lookupRecent(self):
        return self.tf

    def localize(self, laser_frame_scan_locs, odom_to_base, time, weight):
        map_to_base = self.lookupRecent() * odom_to_base
        base_to_map = map_to_base.inv()

        scan_pts, map_pts = np.array(
            self.map.nearestWallptsFromScan(
                laser_frame_scan_locs,
                map_to_base * self.base_to_laser,
            )
        )

        scan_pts_base = base_to_map.inParentArray(scan_pts)
        map_pts_base = base_to_map.inParentArray(map_pts)

        if len(scan_pts) > 10:
            a = np.linalg.norm(map_pts_base - scan_pts_base, axis=1)

            J = (
                np.array(
                    [
                        map_pts_base[:, 0] - scan_pts_base[:, 0],
                        map_pts_base[:, 1] - scan_pts_base[:, 1],
                        map_pts_base[:, 1] * scan_pts_base[:, 0]
                        - map_pts_base[:, 0] * scan_pts_base[:, 1],
                    ]
                ).T
                / a[:, None]
            )

            delta_base = np.linalg.pinv(J.T @ J) @ J.T @ a

            delta_w_base = weight * delta_base

            base_n_to_base = PlanarTransform.basic(
                delta_w_base[0],
                delta_w_base[1],
                delta_w_base[2],
            )

            map_to_base_n = map_to_base * base_n_to_base

            base_to_odom = base_to_map * self.lookupRecent()
            self.set(map_to_base_n * base_to_odom)

        self.broadcast(time)
