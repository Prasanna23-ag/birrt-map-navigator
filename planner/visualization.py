import matplotlib.pyplot as plt
from .map_utils import world_to_map

def visualize(occupancy_map, origin, resolution, start, goal, path, tree_start, tree_goal):
    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy_map, cmap='gray', origin='upper')

    # Plot start and goal
    sx, sy = world_to_map(*start, origin, resolution, occupancy_map.shape)
    gx, gy = world_to_map(*goal, origin, resolution, occupancy_map.shape)
    plt.plot(sx, sy, "go", markersize=8, label="Start")
    plt.plot(gx, gy, "ro", markersize=8, label="Goal")

    # Plot all sampled nodes
    all_nodes = set(tree_start.keys()).union(tree_goal.keys())
    for node in all_nodes:
        mx, my = world_to_map(*node, origin, resolution, occupancy_map.shape)
        plt.plot(mx, my, "b.", markersize=3)

    # Draw tree edges
    for tree in [tree_start, tree_goal]:
        for child, parent in tree.items():
            if parent is not None:
                x1, y1 = world_to_map(*child, origin, resolution, occupancy_map.shape)
                x2, y2 = world_to_map(*parent, origin, resolution, occupancy_map.shape)
                plt.plot([x1, x2], [y1, y2], "c-", linewidth=0.5)

    # Plot final path
    if path:
        px, py = zip(*[world_to_map(*p, origin, resolution, occupancy_map.shape) for p in path])
        plt.plot(px, py, "r-", linewidth=2, label="Bi-RRT Path")

    plt.legend()
    plt.title("Bi-RRT Path Planning with Sampled Nodes and Tree Edges")
    plt.show()
