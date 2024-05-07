import numpy as np
from scipy.spatial.distance import cdist


def find_nearest_obstacle_on_map(occupancy_grid, x_idx, y_idx):
    """
    Find the nearest obstacle on an occupancy grid to a given point.

    Args:
        occupancy_grid (numpy.ndarray): The occupancy grid.
        x_idx (int): The row index of the point.
        y_idx (int): The column index of the point.

    Returns:
        tuple: The coordinates of the nearest obstacle.
    """
    distances = cdist([(x_idx, y_idx)], np.argwhere(occupancy_grid == 0))
    index_of_nearest_obstacle = np.argmin(distances)
    nearest_obstacle = np.argwhere(occupancy_grid == 0)[index_of_nearest_obstacle]

    return nearest_obstacle
