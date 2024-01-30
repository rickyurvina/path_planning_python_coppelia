import math
import numpy as np
from src.components.step1_get_data.traversability import can_traverse_terrain
from src.components.step_3_rrt.find_nearest_obstacle import find_nearest_obstacle


def check_collision(self, node1, node2):
    if node2 is None:
        return False
    points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                         np.linspace(node1.col, node2.col, dtype=int))
    for point in points_between:
        value = self.map_array[point[0]][point[1]]
        if (value > 0 and value < 1) and self.name_method == 'IRRT':
            if not can_traverse_terrain(value):
                self.total_collisions += 1
                return True
        if self.map_array[point[0]][point[1]] == 0:
            self.total_collisions += 1
            return True
    return False
