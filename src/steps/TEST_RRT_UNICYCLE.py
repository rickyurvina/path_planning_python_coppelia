import math

import numpy as np
import matplotlib.pyplot as plt

class UnicycleModel:
    def __init__(self, dt, L):
        self.dt = dt  # Intervalo de tiempo
        self.L = L    # Longitud de la distancia entre ruedas

    def steer(self, state, velocity, steering_angle):
        x, y, theta, v = state

        # Aplicar restricciones cinemáticas
        v = np.clip(velocity, -accel_max, accel_max)
        steering_angle = np.clip(steering_angle, -steer_max, steer_max)

        # Actualizar posición y orientación
        x += v * np.cos(theta) * self.dt
        y += v * np.sin(theta) * self.dt
        theta += (v / self.L) * np.tan(steering_angle) * self.dt

        return np.array([x, y, theta, v])

class RRT:
    def __init__(self, occupancy_grid, start, goal, dt, L, steer_max, accel_max):
        self.occupancy_grid = occupancy_grid
        self.start = start
        self.goal = goal
        self.tree = {tuple(start): None}
        self.model = UnicycleModel(dt, L)
        self.dt = dt
        self.L = L
        self.steer_max = steer_max
        self.accel_max = accel_max

    def plan(self, max_iterations):
        for _ in range(max_iterations):
            new_node = self.sample()
            nearest_node = self.nearest(new_node)
            new_state = self.steer(nearest_node, new_node)
            if not self.collision(new_state):
                self.tree[tuple(new_state)] = nearest_node
                if self.goal_reached(new_state):
                    return self.extract_path(new_state)
        return None

    def sample(self):
        return np.random.rand(4)

    def nearest(self, new_node):
        nearest_node = None
        min_dist = float('inf')
        for node in self.tree:
            dist = np.linalg.norm(np.array(new_node) - np.array(node))
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        return nearest_node

    def steer(self, nearest_node, new_node):
        nearest_state = np.array(nearest_node)
        new_node = np.array(new_node)
        direction = new_node - nearest_state
        direction /= np.linalg.norm(direction)
        delta = np.arctan2(direction[1], direction[0])
        velocity = np.linalg.norm(direction) * self.L * np.tan(delta)
        return self.model.steer(nearest_state, velocity, delta)

    def collision(self, state):
        """
        Verifica si un estado colisiona con un obstáculo en el occupancy grid.

        :param state: Estado a verificar, en formato (x, y, theta, velocity).
        :return: True si hay colisión, False si no.
        """
        x, y, _, _ = state

        # Resolución del occupancy grid (espaciado entre celdas)
        resolution = 0.1  # Ajusta esto según tu mapa y resolución

        # Coordenadas de celda en el occupancy grid
        grid_x = int(x / resolution)
        grid_y = int(y / resolution)

        # Comprueba si la celda en el occupancy grid es un obstáculo (1 para obstáculo, 0 para espacio libre)
        if self.occupancy_grid[grid_x][grid_y] == 1:
            return True  # Colisión con un obstáculo
        else:
            return False  # Sin colisión, espacio libre

    def goal_reached(self, state):
        # Implementa la condición de objetivo alcanzado
        x, y, _, _ = state
        goal_x, goal_y, _, _ = self.goal
        distance = np.linalg.norm([goal_x - x, goal_y - y])
        return distance < 0.1
    def extract_path(self, end_state):
        path = []
        current_state = end_state
        while current_state:
            path.append(current_state)
            current_state = self.tree.get(tuple(current_state))
        path.reverse()
        return path

if __name__ == '__main__':
    dt = 0.05
    L = 0.9
    steer_max = np.deg2rad(40.0)
    curvature_max = math.tan(steer_max) / L
    curvature_max = 1.0 / curvature_max + 1.0
    accel_max = 5.0

    grid_size = (100, 100)

    # Crea un occupancy grid lleno de ceros (espacio libre)
    occupancy_grid = np.zeros(grid_size)

    # Agrega un obstáculo en la mitad del mapa
    obstacle_size = (50, 50)  # Tamaño del obstáculo
    obstacle_position = (25, 25)  # Posición del obstáculo en el mapa
    occupancy_grid[
    obstacle_position[0]: obstacle_position[0] + obstacle_size[0],
    obstacle_position[1]: obstacle_position[1] + obstacle_size[1],
    ] = 1  # Marca las celdas del obstáculo con 1 (espacio ocupado)

    start = [0.0, 0.0, 0.0, 0.0]
    goal = [10.0, 10.0, 0.0, 0.0]

    rrt = RRT(occupancy_grid, start, goal, dt, L, steer_max, accel_max)
    path = rrt.plan(1000)

    if path:
        print("Path found!")
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b-')
        plt.axis("equal")
        plt.grid(True)
        plt.show()
    else:
        print("No path found.")
