import numpy as np
from scipy.spatial import cKDTree
from PlanarTransform import PlanarTransform

WALLTHRESHOLD = 50
MAXDISTANCE = 0.5
MAXPTS = 50


class Map:
    def __init__(self, map, scale=1, map_to_grid=PlanarTransform.unity()) -> None:
        self.shape = map.shape
        self.map = map
        self.wallpts = self.getWallpts(map)

        self.walltree = cKDTree(self.wallpts)
        self.scale = scale

        self.map_to_grid = map_to_grid

    def getWallpts(self, map):
        wallpts = np.zeros((0, 2), dtype=np.int)
        for v in range(map.shape[0]):
            for u in range(map.shape[1]):
                if map[v, u] > WALLTHRESHOLD:
                    # Also check the adjacent pixels in a 3x3 grid.
                    adjacent = map[
                        max(0, v - 1) : min(map.shape[0], v + 2),
                        max(0, u - 1) : min(map.shape[1], u + 2),
                    ]
                    if not np.all(adjacent > WALLTHRESHOLD):
                        wallpts = np.vstack([wallpts, np.array([u, v])])
        return wallpts

    def gridToMapFrame(self, pts):
        spts = (pts) * self.scale
        return self.map_to_grid.inParentArray(spts)

    def nearestWallpts(self, pts, max_dist=MAXDISTANCE):
        distances, indices = self.walltree.query(pts)

        isgood = np.where(distances * self.scale < max_dist)
        good_indices = indices[isgood]

        wall_pts = self.gridToMapFrame(self.walltree.data[good_indices, :])

        return wall_pts, isgood

    def filterScan(
        self, distances, angle_min, angle_max, range_min, range_max, max_pts=MAXPTS
    ):
        d = np.array(distances)
        ts = np.linspace(angle_min, angle_max, len(d))

        distance_filter = np.logical_and(d > range_min, d < range_max)

        indices = np.array(list(range(len(distance_filter))))[distance_filter]
        if len(indices) > max_pts:
            remove_indices = indices[
                np.round(
                    np.linspace(0, len(indices) - 1, len(indices) - max_pts)
                ).astype(int)
            ]
            distance_filter[remove_indices] = False

        d = d[distance_filter].reshape((-1, 1))
        ts = ts[distance_filter].reshape((-1, 1))

        xs = d * np.cos(ts)
        ys = d * np.sin(ts)

        return np.hstack((xs, ys))

    def nearestWallptsFromScan(self, laser_frame_scan_locs, map_to_laser):
        grid_to_laser = self.map_to_grid.inv() * map_to_laser

        grid_frame_scan_locs = grid_to_laser.inParentArray(laser_frame_scan_locs)

        grid_frame_pts = np.array(grid_frame_scan_locs) / self.scale
        grid_frame_pxls = np.round(grid_frame_pts)

        fits_in_map = np.logical_and.reduce(
            np.array(
                [
                    grid_frame_pxls[:, 0] < self.shape[0],
                    grid_frame_pxls[:, 0] >= 0,
                    grid_frame_pxls[:, 1] < self.shape[1],
                    grid_frame_pxls[:, 1] >= 0,
                ]
            )
        )

        wall_pts, close_to_map = self.nearestWallpts(grid_frame_pxls[fits_in_map])

        return (
            self.map_to_grid.inParentArray(
                grid_frame_pts[fits_in_map][close_to_map] * self.scale
            ),
            wall_pts,
        )

    def inFreespace(self, pts):
        grid_frame_locs = self.map_to_grid.inv().inParentArray(pts)
        grid_frame_pts = np.array(grid_frame_locs) / self.scale
        grid_frame_pxls = np.round(grid_frame_pts)

        fits_in_map = np.logical_and.reduce(
            np.array(
                [
                    grid_frame_pxls[:, 0] < self.shape[0],
                    grid_frame_pxls[:, 0] >= 0,
                    grid_frame_pxls[:, 1] < self.shape[1],
                    grid_frame_pxls[:, 1] >= 0,
                ]
            )
        )

        res = np.array([0] * len(pts), dtype=bool)
        nonzero = np.nonzero(fits_in_map)
        idx = grid_frame_pxls[nonzero].astype(int)
        res[nonzero] = np.logical_and(
            self.map[idx[:, 1], idx[:, 0]] < WALLTHRESHOLD,
            self.map[idx[:, 1], idx[:, 0]] >= 0,
        )

        return res

    def distancesToNearestWall(self, pts):
        grid_frame_locs = self.map_to_grid.inv().inParentArray(pts)

        grid_frame_pts = np.array(grid_frame_locs) / self.scale
        grid_frame_pxls = np.round(grid_frame_pts)

        fits_in_map = np.logical_and.reduce(
            np.array(
                [
                    grid_frame_pxls[:, 0] < self.shape[0],
                    grid_frame_pxls[:, 0] >= 0,
                    grid_frame_pxls[:, 1] < self.shape[1],
                    grid_frame_pxls[:, 1] >= 0,
                ]
            )
        )

        nonzero = np.nonzero(fits_in_map)
        wall_pts, _ = self.nearestWallpts(
            grid_frame_pxls[fits_in_map], max_dist=float("inf")
        )

        dists = np.ones(len(pts))
        dists[nonzero] = np.linalg.norm(pts[fits_in_map] - wall_pts, axis=1)
        return dists


if __name__ == "__main__":
    init_map = np.zeros((10, 10))
    init_map[0, 0] = 100
    init_map[5, 5] = 100
    init_map[9, 9] = 100

    map = Map(init_map)

    print(map.nearestWallpts(np.array([[4, 4], [2, 2], [2, 10]])))
