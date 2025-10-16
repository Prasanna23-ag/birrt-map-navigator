import numpy as np
from .map_utils import map_to_world, world_to_map
from .collision import is_free

def sample_free_point(occupancy_map, bounds, origin, resolution, buffer=5, min_distance=None, reference_point=None, max_attempts=10000):
    for _ in range(max_attempts):
        mx = np.random.randint(bounds["xmin"] + buffer, bounds["xmax"] - buffer)
        my = np.random.randint(bounds["ymin"] + buffer, bounds["ymax"] - buffer)
        if is_free(mx, my, occupancy_map):
            wx, wy = map_to_world(mx, my, origin, resolution, occupancy_map.shape)
            if reference_point and min_distance:
                if np.linalg.norm(np.array([wx, wy]) - np.array(reference_point)) < min_distance:
                    continue
            return [wx, wy]
    raise RuntimeError("No free space found inside map bounds.")
