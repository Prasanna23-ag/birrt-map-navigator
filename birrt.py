from planner.map_utils import load_map, inflate_map, world_to_map
from planner.sampling import sample_free_point
from planner.collision import is_collision_free
from planner.birrt_planner import build_birrt
from planner.visualization import visualize

# === Load and preprocess map ===
map_img, resolution, origin, map_bounds = load_map('map2.pgm', 'map2.yaml')
occupancy_map = inflate_map(map_img, resolution, robot_radius=0.18, safety_margin=0.0)

# === Run planner ===
start = sample_free_point(occupancy_map, map_bounds, origin, resolution, buffer=5)
goal = sample_free_point(occupancy_map, map_bounds, origin, resolution, buffer=5, min_distance=0.5, reference_point=start)
path, tree_start, tree_goal = build_birrt(start, goal, occupancy_map, map_bounds, origin, resolution)

# === Visualize result ===
visualize(occupancy_map, origin, resolution, start, goal, path, tree_start, tree_goal)
