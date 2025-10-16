import numpy as np
from .map_utils import world_to_map

def is_free(mx, my, occupancy_map):
    return 0 <= mx < occupancy_map.shape[1] and 0 <= my < occupancy_map.shape[0] and occupancy_map[my, mx] == 1

def is_collision_free(p1, p2, occupancy_map, origin, resolution):
    dist = np.linalg.norm(np.array(p2) - np.array(p1))
    steps = max(10, int(dist / (resolution * 0.5)))
    for i in range(steps + 1):
        x = p1[0] + (p2[0] - p1[0]) * i / steps
        y = p1[1] + (p2[1] - p1[1]) * i / steps
        mx, my = world_to_map(x, y, origin, resolution, occupancy_map.shape)
        if not is_free(mx, my, occupancy_map):
            return False
    return True
