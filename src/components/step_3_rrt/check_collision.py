import math

import numpy as np

from src.components.step1_get_data.traversability import can_traverse_terrain


def check_collision(self, node1, node2):
    if node2 is None:
        return False, None
    # closest_obstacle_distance = float('inf')
    points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                         np.linspace(node1.col, node2.col, dtype=int))
    for point in points_between:
        # col, row = point
        value = self.map_array[point[0]][point[1]]
        if (value > 0 and value < 1) and self.name_method == 'IRRT':
            if not can_traverse_terrain(value):
                self.total_collisions += 1
                return True, None
        # obstacle_distance = math.sqrt((node2.row - row) ** 2 + (node2.col - col) ** 2)
        # closest_obstacle_distance = min(closest_obstacle_distance, obstacle_distance)

        if self.map_array[point[0]][point[1]] == 0:
            self.total_collisions += 1
            return True, None
    return False, None
