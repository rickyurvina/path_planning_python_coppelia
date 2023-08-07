class ConfigurationSpace:
    def __init__(self, occupancy_grid, robot_radius):
        self.occupancy_grid = occupancy_grid
        self.robot_radius = robot_radius
        self.x_min, self.x_max, self.y_min, self.y_max = self.calculate_limits()

    def calculate_limits(self):
        # Calcular los límites del espacio de configuración en función del tamaño del occupancy grid
        # y las dimensiones físicas del robot
        x_min = 0
        x_max = len(self.occupancy_grid[0]) - 1
        y_min = 0
        y_max = len(self.occupancy_grid) - 1
        return x_min, x_max, y_min, y_max

    def is_valid_configuration(self, x, y):
        # Verificar si la configuración (x, y) es válida en el occupancy grid
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return False  # Fuera de los límites del occupancy grid
        if self.occupancy_grid[y][x] == 1:
            return False  # Dentro de un obstáculo
        return True

# Ejemplo de uso
if __name__ == "__main__":
    # Supongamos que tienes un occupancy grid representado como una matriz donde 0 es espacio libre y 1 es obstáculo
    occupancy_grid = [
        [0, 0, 0, 1, 1],
        [0, 1, 0, 0, 1],
        [0, 0, 0, 1, 0],
        [1, 0, 1, 1, 0],
        [1, 0, 0, 0, 0],
    ]
    robot_radius = 0.5  # Radio del robot uniciclo
    config_space = ConfigurationSpace(occupancy_grid, robot_radius)

    # Verificar si una configuración (x, y) es válida
    x = 2
    y = 3
    if config_space.is_valid_configuration(x, y):
        print(f"La configuración ({x}, {y}) es válida.")
    else:
        print(f"La configuración ({x}, {y}) no es válida debido a un obstáculo o está fuera del espacio.")
