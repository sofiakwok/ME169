import numpy as np


def brushFireStep(map):
    dxs = [-1, 0, 1]
    dys = [-1, 0, 1]

    old_map = np.copy(map)
    for x in range(old_map.shape[0]):
        for y in range(old_map.shape[1]):
            if old_map[x, y] is not None:
                for dx in dxs:
                    for dy in dys:
                        if (
                            x + dx < old_map.shape[0]
                            and x + dx >= 0
                            and y + dy < old_map.shape[1]
                            and y + dy >= 0
                            and old_map[x + dx, y + dy] is None
                        ):
                            map[x + dx, y + dy] = map[x, y]


def brushFireInit(map, min_occupied):
    out = np.empty(map.shape, dtype=object)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i, j] > min_occupied:
                out[i, j] = (i, j)

    return out


def getNearestWalls(map, min_occupied):
    out = brushFireInit(map, min_occupied)
    while np.any(out == None):
        brushFireStep(out)
    return out


if __name__ == "__main__":
    map = np.zeros((10, 10))
    map[0, 0] = 100
    map[5, 5] = 100
    map[9, 9] = 100

    print(getNearestWalls(map, 50))
