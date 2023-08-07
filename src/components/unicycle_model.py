import random
import math
import matplotlib.pyplot as plt
import numpy as np
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

class ConfigurationSpace:
    def __init__(self, occupancy_grid, robot_radius):
        self.occupancy_grid = occupancy_grid
        self.robot_radius = robot_radius
        self.x_min, self.x_max, self.y_min, self.y_max = self.calculate_limits()

    def calculate_limits(self):
        x_min = 0
        x_max = len(self.occupancy_grid[0]) - 1
        y_min = 0
        y_max = len(self.occupancy_grid) - 1
        return x_min, x_max, y_min, y_max

    def is_valid_configuration(self, x, y):
        x = int(x)  # Convertir a entero
        y = int(y)
        # Verificar si la configuración (x, y) es válida en el occupancy grid

        if x < 0 or x >= len(self.occupancy_grid[0]) or y < 0 or y >= len(self.occupancy_grid):
            return False  # Fuera de los límites del occupancy grid
        if self.occupancy_grid[y][x] == 0:
            return False
        return True


# ... (implementación de la clase ConfigurationSpace)

# Función para generar un nuevo nodo aleatorio en el espacio de configuración
def generate_random_node(config_space):
    x = random.uniform(config_space.x_min, config_space.x_max)
    y = random.uniform(config_space.y_min, config_space.y_max)
    return Node(x, y)


# Función para verificar si un nodo es válido cinemáticamente
def is_valid_node(node, config_space):
    if not config_space.is_valid_configuration(node.x, node.y):
        return False
    # Otras comprobaciones cinemáticas específicas del robot, si es necesario
    return True


# Función para encontrar el nodo más cercano en el árbol a un nodo dado
def find_nearest_node(tree, node):
    nearest_node = None
    nearest_dist = float('inf')
    for n in tree:
        dist = math.hypot(n.x - node.x, n.y - node.y)
        if dist < nearest_dist:
            nearest_node = n
            nearest_dist = dist
    return nearest_node


# Función para extender el árbol desde el nodo más cercano al nuevo nodo
def extend_tree(from_node, to_node, config_space):
    # Paso de extensión (puedes usar una interpolación cinemática para extender el árbol de manera más suave)
    extend_distance = 0.5  # Distancia de extensión
    d, theta = calc_distance_and_angle(from_node, to_node)
    if d > extend_distance:
        d = extend_distance
    new_node = Node(from_node.x + d * math.cos(theta), from_node.y + d * math.sin(theta))
    new_node.parent = from_node
    if is_valid_node(new_node, config_space):
        return new_node
    return None

def steer(from_node, to_node, config_space):
    # Calculate the distance and angle between from_node and to_node
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    # Define the step size for steering
    step_size = 0.5
    # Calculate the number of steps needed to reach the new node
    num_steps = int(d / step_size)
    # Calculate the new node's position for each step
    for i in range(num_steps):
        x = from_node.x + step_size * math.cos(theta)
        y = from_node.y + step_size * math.sin(theta)
        new_node = Node(x, y)
        if not is_valid_node(new_node, config_space):
            # If the new node is invalid, return the last valid node
            return from_node
        from_node = new_node

    return from_node
# Función RRT* para encontrar un camino factible
def rrt_star(start, goal, config_space, max_iter=500):
    tree = [start]

    for _ in range(max_iter):
        print("Iteración: ", _)
        random_node = generate_random_node(config_space)
        nearest_node = find_nearest_node(tree, random_node)
        new_node = steer(nearest_node, random_node, config_space)

        if is_valid_node(new_node, config_space):
            new_node.cost = nearest_node.cost + calc_distance(nearest_node, new_node)
            tree.append(new_node)
            tree = rewire_tree(tree, new_node, config_space)

            if math.hypot(new_node.x - goal.x, new_node.y - goal.y) < 1.0:
                goal.parent = new_node
                goal.cost = new_node.cost + calc_distance(new_node, goal)
                tree.append(goal)
                for _ in range(50):
                    # Seleccionar un nodo aleatorio del árbol
                    random_tree_node = random.choice(tree)

                    # Intentar ajustar su padre para mejorar el camino
                    potential_parent = steer(random_tree_node, new_node, config_space)

                    # Si el ajuste mejora el costo, actualizar el nodo y recalcular el costo para los nodos afectados
                    if potential_parent and potential_parent.cost + calc_distance(potential_parent,
                                                                                  new_node) < new_node.cost:
                        new_node.parent = potential_parent
                        new_node.cost = potential_parent.cost + calc_distance(potential_parent, new_node)

                        # Recalcular el costo de los nodos afectados por el cambio
                        tree = rewire_tree(tree, new_node, config_space)

                return tree

    return None  # No se encontró un camino


# Función para calcular la distancia entre dos nodos
def calc_distance(node1, node2):
    return math.hypot(node1.x - node2.x, node1.y - node2.y)


# Función para calcular la distancia y el ángulo entre dos nodos
def calc_distance_and_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    return d, theta


# Función para reorganizar el árbol utilizando el algoritmo RRT*
def rewire_tree(tree, new_node, config_space):
    # Buscar nodos en el árbol que puedan tener un camino más corto al nuevo nodo


    return tree

def plot_path(occupancy_grid, path):
    """
    Plot the occupancy grid and the path on it.

    Parameters:
        occupancy_grid (list of lists): 2D matrix representing the occupancy grid.
        path (list of tuples): List of (x, y) coordinates representing the path.

    Returns:
        None
    """
    occupancy_grid = np.array(occupancy_grid)
    path_coordinates = [(node.x, node.y) for node in path]

    fig, ax = plt.subplots()
    ax.imshow(occupancy_grid, cmap='gray', origin='lower')
    if path_coordinates:
        path_x, path_y = zip(*path_coordinates)
        ax.plot(path_x, path_y, color='red', linewidth=2)

    ax.set_xlim(0, occupancy_grid.shape[1] - 1)
    ax.set_ylim(0, occupancy_grid.shape[0] - 1)
    ax.set_aspect('equal')

    plt.show()
# Ejemplo de uso
if __name__ == "__main__":
    # Supongamos que tienes un occupancy grid representado como una matriz donde 0 es espacio libre y 1 es obstáculo
    occupancy_grid = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [1, 0, 1, 1, 0],
        [1, 0, 0, 0, 0],
    ]
    robot_radius = 0.1  # Radio del robot uniciclo
    config_space = ConfigurationSpace(occupancy_grid, robot_radius)

    # Definir el nodo inicial y el nodo objetivo
    start = Node(2, 3)
    goal = Node(3, 2)

    # Ejecutar el algoritmo RRT*
    path = rrt_star(start, goal, config_space)

    if path:
        print("Camino encontrado:")
        plot_path(occupancy_grid, path)
    else:
        print("No se encontró un camino factible.")
