import numpy as np
from .collision import is_collision_free
from .sampling import sample_free_point


def build_birrt(start, goal, occupancy_map, bounds, origin, resolution, max_iters=3000, step_size=0.1, connect_threshold=0.5):
    def steer(p1, p2, step):
        direction = np.array(p2) - np.array(p1)
        length = np.linalg.norm(direction)
        return (np.array(p1) + step * direction / length).tolist() if length > 0 else p1

    def nearest(tree, point):
        return min(tree, key=lambda n: np.linalg.norm(np.array(n) - np.array(point)))

    tree_start, tree_goal = {tuple(start): None}, {tuple(goal): None}
    for _ in range(max_iters):
        rand = sample_free_point(occupancy_map, bounds, origin, resolution)
        for tree_a, tree_b in [(tree_start, tree_goal), (tree_goal, tree_start)]:
            nearest_node = nearest(tree_a, rand)
            new_node = steer(nearest_node, rand, step_size)
            if is_collision_free(nearest_node, new_node, occupancy_map, origin, resolution):
                tree_a[tuple(new_node)] = nearest_node
                connect_node = nearest(tree_b, new_node)
                if np.linalg.norm(np.array(connect_node) - np.array(new_node)) < connect_threshold:
                    if is_collision_free(new_node, connect_node, occupancy_map, origin, resolution):
                        path_a, path_b = [], []
                        node = new_node
                        while node: path_a.append(node); node = tree_a[tuple(node)]
                        node = connect_node
                        while node: path_b.append(node); node = tree_b[tuple(node)]
                        if tree_a is tree_goal: path_a, path_b = path_b, path_a
                        return path_a[::-1] + path_b, tree_start, tree_goal
    return None, tree_start, tree_goal
