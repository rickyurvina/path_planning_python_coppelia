import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import euclidean
from src.components.loadData import return_ocuppancy_grid
mapa_local = return_ocuppancy_grid()['occupancy_grid']
# mapa_local = return_ocuppancy_grid()['mapa_local']
start = (200, 180)
goal = (400, 180)

# Definir la clase Node para representar un nodo en el árbol del RRT*
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0


# Definir una función que comprueba si un punto está en una posición libre del mapa
def collision_free(point, mapa_local):
    x, y = point
    if mapa_local[int(y), int(x)] == 0:
        return True
    return False


# Definir una función que implementa el algoritmo RRT*
def rrt_star(start, goal, mapa_local, max_iters=10000, step_size=20):
    nodes = []
    nodes.append(Node(start[0], start[1]))
    for i in range(max_iters):
        x_rand = np.random.uniform(0, mapa_local.shape[1])
        y_rand = np.random.uniform(0, mapa_local.shape[0])
        point_rand = [x_rand, y_rand]
        nearest_node = nodes[0]
        for node in nodes:
            if euclidean([node.x, node.y], point_rand) < euclidean([nearest_node.x, nearest_node.y], point_rand):
                nearest_node = node
        x_new = nearest_node.x + step_size * (point_rand[0] - nearest_node.x) / euclidean(
            [nearest_node.x, nearest_node.y], point_rand)
        y_new = nearest_node.y + step_size * (point_rand[1] - nearest_node.y) / euclidean(
            [nearest_node.x, nearest_node.y], point_rand)
        point_new = [x_new, y_new]
        if not collision_free(point_new, mapa_local):
            continue
        nearest_neighbors = []
        for node in nodes:
            if euclidean([node.x, node.y], point_new) < step_size:
                nearest_neighbors.append(node)
        costs = []
        for neighbor in nearest_neighbors:
            cost = neighbor.cost + euclidean([neighbor.x, neighbor.y], point_new)
            costs.append(cost)
        min_cost = min(costs)
        min_node = nearest_node
        for neighbor, cost in zip(nearest_neighbors, costs):
            if neighbor.cost + euclidean([neighbor.x, neighbor.y], point_new) == min_cost:
                min_node = neighbor
        nodes.append(Node(point_new[0], point_new[1]))
        nodes[-1].parent = min_node
        nodes[-1].cost = min_cost
        for neighbor in nearest_neighbors:
            if neighbor.cost > min_cost + euclidean([neighbor.x, neighbor.y], point_new):
                neighbor.cost = min_cost + euclidean([neighbor.x, neighbor.y], point_new)
                neighbor.parent = nodes[-1]

    path = []
    current_node = nodes[-1]
    while current_node is not None:
        path.append([current_node.x, current_node.y])
        current_node = current_node.parent
    path = np.array(path)
    return path


# Realizar el path planning con RRT*
path = rrt_star(start, goal, mapa_local)

# Graficar el mapa y el camino generado por RRT*
fig, ax = plt.subplots(figsize=(10,10))
ax.imshow(mapa_local, cmap='gray', origin='lower')
ax.plot(start[1], start[0], 'ro', markersize=10)
ax.plot(goal[1], goal[0], 'bo', markersize=10)
ax.plot(path[:,1], path[:,0], 'r', linewidth=2)
plt.show()
