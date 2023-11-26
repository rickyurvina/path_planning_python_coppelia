import numpy as np


def check_collision(self, node1, node2):
    if node2 is None:
        return False
    points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                         np.linspace(node1.col, node2.col, dtype=int))
    for point in points_between:
        if self.map_array[point[0]][point[1]] < 1:
            self.total_collisions += 1
            return True
    return False
