import cv2
import yaml
import numpy as np
from scipy.ndimage import binary_dilation

def load_map(pgm_path, yaml_path):
    map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)
    resolution = metadata['resolution']
    origin = metadata['origin'][:2]

    contours, _ = cv2.findContours((map_img == 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    bounds = {"xmin": x + 1, "xmax": x + w - 2, "ymin": y + 1, "ymax": y + h - 2}
    return map_img, resolution, origin, bounds

def inflate_map(map_img, resolution, robot_radius, safety_margin):
    occupied = (map_img == 0)
    inflation_radius = max(1, int((robot_radius + safety_margin) / resolution))
    inflated = binary_dilation(occupied, iterations=inflation_radius)
    occupancy_map = np.ones_like(map_img, dtype=np.uint8)
    occupancy_map[inflated] = 0
    return occupancy_map

def world_to_map(x, y, origin, resolution, map_shape):
    mx = int((x - origin[0]) / resolution)
    my = map_shape[0] - int((y - origin[1]) / resolution)
    return mx, my

def map_to_world(mx, my, origin, resolution, map_shape):
    x = mx * resolution + origin[0]
    y = (map_shape[0] - my) * resolution + origin[1]
    return x, y
