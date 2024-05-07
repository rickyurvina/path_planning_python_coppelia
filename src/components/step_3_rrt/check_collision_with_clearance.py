from src.components.step1_get_data.traversability import can_traverse_terrain
from src.components.step_3_rrt.check_collision import check_collision
from src.steps import config


def check_collision_with_clearance(self, node1, node2, clearance_radius=config.CLEARANCE_RADIUS):
    """
    Check for collision between two nodes with clearance.

    Args:
        self: Reference to the class instance.
        node1 (Node): Starting node.
        node2 (Node): Ending node.
        clearance_radius (int, optional): Radius of clearance around node2. Defaults to config.CLEARANCE_RADIUS.

    Returns:
        bool: True if collision detected, False otherwise.
    """
    if node2 is None:
        return False

    # Check for collision along the line between the two nodes
    collision = check_collision(self, node1, node2)

    if collision:
        return True

    # Check for collision in the circular area around node2
    for i in range(-clearance_radius, clearance_radius + 1):
        for j in range(-clearance_radius, clearance_radius + 1):
            row = int(node2.row) + i
            col = int(node2.col) + j
            if not (0 <= row < self.size_y_max and 0 <= col < self.size_x_max):
                continue  # Skip if we are outside the map boundaries

            value = self.map_array[row][col]
            if value > 0 and value < 1:
                if not can_traverse_terrain(value):
                    self.total_collisions += 1
                    return True
            if value == 0:
                self.total_collisions += 1
                return True

    return False
