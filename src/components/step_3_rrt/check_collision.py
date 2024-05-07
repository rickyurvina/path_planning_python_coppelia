import numpy as np
from src.components.step1_get_data.traversability import can_traverse_terrain


def check_collision(self, node1, node2):
    """
    Check for collision between two nodes.

    Args:
        self: Reference to the class instance.
        node1 (Node): Starting node.
        node2 (Node): Ending node.

    Returns:
        bool: True if collision detected, False otherwise.
    """
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
