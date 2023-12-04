from src.components.step1_get_data.traversability import can_traverse_terrain
from src.components.step_3_rrt.check_collision import check_collision
from src.steps import config


def check_collision_with_clearance(self, node1, node2, clearance_radius=config.CLEARANCE_RADIUS):
    if node2 is None:
        return False

    # Verificar colisión en la línea entre los dos nodos
    if check_collision(self, node1, node2):
        return True

    # Verificar colisión en el área circular alrededor de node2
    for i in range(-clearance_radius, clearance_radius + 1):
        for j in range(-clearance_radius, clearance_radius + 1):
            row = int(node2.row) + i
            col = int(node2.col) + j
            if not (0 <= row < self.size_y_max and 0 <= col < self.size_x_max):
                continue  # Saltear si estamos fuera de los límites del mapa

            value = self.map_array[row][col]
            if value > 0 and value < 1:
                if not can_traverse_terrain(value):
                    self.total_collisions += 1
                    return True
            if value == 0:
                self.total_collisions += 1
                return True

    return False
