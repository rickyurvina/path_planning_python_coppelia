import numpy as np
from scipy.spatial.distance import cdist


def find_nearest_obstacle_on_map(occupancy_grid, x_idx, y_idx):
    distances = cdist([(x_idx, y_idx)], np.argwhere(occupancy_grid == 0))
    index_of_nearest_obstacle = np.argmin(distances)
    nearest_obstacle = np.argwhere(occupancy_grid == 0)[index_of_nearest_obstacle]

    return nearest_obstacle
