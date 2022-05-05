import numpy as np
from scipy.spatial import cKDTree
from PlanarTransform import PlanarTransform

WALLTHRESHOLD = 50
MAXDISTANCE = 5


class Map:
    def __init__(self, map, scale=1, map_to_grid=PlanarTransform.unity()) -> None:
        self.shape = map.shape
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

    def gridToMapFrame(self, pt):
        spt = (pt + 0.5) * self.scale
        x, y = self.map_to_grid.inParent(spt[0], spt[1])
        return np.array([x, y])

    def nearestWallpts(self, pts):
        distances, indices = self.walltree.query(pts)

        isgood = np.where(distances * self.scale < MAXDISTANCE)
        good_indices = indices[isgood]

        good_pts = np.apply_along_axis(self.gridToMapFrame, 1, pts[isgood])
        wall_pts = np.apply_along_axis(
            self.gridToMapFrame, 1, self.walltree.data[good_indices, :]
        )

        return (good_pts, wall_pts)

    def nearestWallptsFromScan(
        self, distances, angle_min, angle_max, range_min, range_max, map_to_laser
    ):
        grid_to_laser = self.map_to_grid.inv() * map_to_laser

        nd = len(distances)
        grid_pts = []
        for i in range(0, nd, 10):
            if distances[i] > range_min and distances[i] < range_max:
                d = distances[i]
                t = (angle_max - angle_min) * (i / (nd - 1)) + angle_min
                gx, gy = grid_to_laser.inParent(d * np.cos(t), d * np.sin(t))
                grid_pts.append([gx, gy])

        pts = np.round(np.array(grid_pts) / self.scale)
        good_pts = pts[
            np.logical_and.reduce(
                np.array(
                    [
                        pts[:, 0] < self.shape[0],
                        pts[:, 0] >= 0,
                        pts[:, 1] < self.shape[1],
                        pts[:, 1] >= 0,
                    ]
                )
            )
        ]

        return self.nearestWallpts(good_pts)


if __name__ == "__main__":
    init_map = np.zeros((10, 10))
    init_map[0, 0] = 100
    init_map[5, 5] = 100
    init_map[9, 9] = 100

    map = Map(init_map)

    print(map.nearestWallpts(np.array([[4, 4], [2, 2], [2, 10]])))
