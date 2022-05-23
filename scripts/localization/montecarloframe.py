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

import map as rmap

CONF_CHANGE_CONST = 0.1
BEST_CONF_DURATION = 5


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

        self.last_localization_time = rospy.Time.now()

        self.conf = 0.5
        self.best_prev_conf = self.conf
        self.best_prev_conf_time = rospy.Time.now()

    def randomize(self):
        self.tf = PlanarTransform.basic(
            random.uniform(-3, 3), random.uniform(3, -1), random.uniform(0, 2 * np.pi)
        )

        self.conf = 0.5
        self.best_prev_conf = 0.5
        self.best_prev_conf_time = rospy.Time.now()

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
        dt = (time - self.last_localization_time).to_sec()
        map_to_base = self.lookupRecent() * odom_to_base
        base_to_map = map_to_base.inv()

        scan_pts, map_pts = np.array(
            self.map.nearestWallptsFromScan(
                laser_frame_scan_locs, map_to_base * self.base_to_laser
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

            delta_w_base = weight * delta_base * max(1, dt)

            base_n_to_base = PlanarTransform.basic(
                delta_w_base[0], delta_w_base[1], delta_w_base[2]
            )

            map_to_base_n = map_to_base * base_n_to_base

            base_to_odom = base_to_map * self.lookupRecent()
            self.set(map_to_base_n * base_to_odom)

            conf_dists = np.mean(np.linalg.norm(scan_pts_base - map_pts_base, axis=1))

            conf_dists_with_missed = (
                -(
                    rmap.MAXDISTANCE * (len(laser_frame_scan_locs) - len(scan_pts_base))
                    + conf_dists * len(scan_pts_base)
                )
                / len(laser_frame_scan_locs)
                / rmap.MAXDISTANCE
            ) + 1

            self.conf = (
                self.conf * (1 - CONF_CHANGE_CONST * dt)
                + conf_dists_with_missed * CONF_CHANGE_CONST * dt
            )

        else:
            self.conf = self.conf * (1 - CONF_CHANGE_CONST * dt)

        if (self.conf > self.best_prev_conf) or (
            time - self.best_prev_conf_time
        ).to_sec() > BEST_CONF_DURATION:
            self.best_prev_conf = self.conf
            self.best_prev_conf_time = time

        self.last_localization_time = time
        self.broadcast(time)
