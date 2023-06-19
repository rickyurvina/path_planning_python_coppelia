import numpy as np
import matplotlib.pyplot as plt
from loadOcuppancyGrid import return_ocuppancy_grid

occupancy_grid = return_ocuppancy_grid()['occupancy_grid']
mapa_local = return_ocuppancy_grid()['mapa_local']


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRTStar:
    def __init__(self, start, goal, obstacle_grid, expand_dist=1.0, max_iter=10000, goal_sample_rate=0.1, radius=10.0):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_grid = obstacle_grid
        self.expand_dist = expand_dist
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.radius = radius
        self.node_list = []

    def generate_path(self):
        self.node_list = [self.start]
        for _ in range(self.max_iter):
            if np.random.rand() > self.goal_sample_rate:
                rnd_node = self.random_node()
            else:
                rnd_node = self.goal

            nearest_node = self.nearest_node(rnd_node)
            new_node = self.steer(nearest_node, rnd_node)
            if self.obstacle_free(nearest_node, new_node):
                near_nodes = self.near_nodes(new_node)
                min_cost_node = nearest_node
                min_cost = self.cost_to_node(nearest_node) + self.distance(nearest_node, new_node)
                for node in near_nodes:
                    if self.obstacle_free(node, new_node) and self.cost_to_node(node) + self.distance(node,
                                                                                                      new_node) < min_cost:
                        min_cost_node = node
                        min_cost = self.cost_to_node(node) + self.distance(node, new_node)
                new_node.parent = min_cost_node
                self.node_list.append(new_node)

                self.rewire(new_node, near_nodes)

                if self.distance(new_node, self.goal) <= self.expand_dist:
                    final_node = self.steer(new_node, self.goal)
                    if self.obstacle_free(new_node, self.goal):
                        final_node.parent = new_node
                        self.node_list.append(final_node)
                        return self.extract_path(final_node)

        return None

    def random_node(self):
        x = np.random.uniform(low=0, high=self.obstacle_grid.shape[0])
        y = np.random.uniform(low=0, high=self.obstacle_grid.shape[1])
        return Node(x, y)

    def nearest_node(self, node):
        distances = [self.distance(node, n) for n in self.node_list]
        min_index = np.argmin(distances)
        return self.node_list[min_index]

    def steer(self, from_node, to_node):
        if self.distance(from_node, to_node) < self.expand_dist:
            return to_node
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        x = from_node.x + self.expand_dist * np.cos(theta)
        y = from_node.y + self.expand_dist * np.sin(theta)
        return Node(x, y)

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def obstacle_free(self, from_node, to_node):
        x0 = int(from_node.x)
        y0 = int(from_node.y)
        x1 = int(to_node.x)
        y1 = int(to_node.y)
        steep = abs(y1 - y0) > abs(x1 - x0)
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
        swapped = x0 > x1
        if swapped:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
        dx = x1 - x0
        dy = abs(y1 - y0)
        error = dx / 2
        ystep = 1 if y0 < y1 else -1
        y = y0
        for x in range(x0, x1 + 1):
            if steep:
                if self.obstacle_grid[x, y] == 0:  # Cambiar de 1 a 0
                    return False
            else:
                if self.obstacle_grid[y, x] == 0:  # Cambiar de 1 a 0
                    return False
            error -= dy
            if error < 0:
                y += ystep
                error += dx
        return True

    def near_nodes(self, node):
        n = len(self.node_list)
        r = min(self.radius * np.sqrt(np.log(n) / n), self.expand_dist)
        distances = [self.distance(node, n) for n in self.node_list]
        near_indices = [i for i, d in enumerate(distances) if d <= r]
        near_nodes = [self.node_list[i] for i in near_indices]
        return near_nodes

    def cost_to_node(self, node):
        cost = 0.0
        while node.parent is not None:
            cost += self.distance(node, node.parent)
            node = node.parent
        return cost

    def rewire(self, node, near_nodes):
        for near_node in near_nodes:
            if self.cost_to_node(node) + self.distance(node, near_node) < self.cost_to_node(near_node):
                if self.obstacle_free(near_node, node):
                    near_node.parent = node

    def extract_path(self, final_node):
        path = []
        node = final_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path


def main_rrt():
    try:
        obstacle_grid = np.ones((1000, 1000))
        obstacle_grid[400:600, 400:600] = 0
        start = (210, 50)
        goal = (400, 400)

        # Crear el planificador RRT*
        rrt_star = RRTStar(start, goal, mapa_local)

        # Generar el camino
        path = rrt_star.generate_path()

        # Visualizar el occupancy grid y el camino
        plt.imshow(mapa_local, cmap='gray', origin='lower')
        plt.plot(start[0], start[1], 'ro', label='Start')
        plt.plot(goal[0], goal[1], 'go', label='Goal')
        if path is not None:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, 'b-', label='Path')
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('RRT* Path Planning')
        plt.show()

    except Exception as e:
        print('Error RRT', e)

# main_rrt()

# Node.__init__(): O(1) (operación constante).
#
# RRTStar.__init__(): O(1) (operación constante).
#
# RRTStar.generate_path(): El costo computacional de esta función depende de los parámetros establecidos y de la cantidad de iteraciones (max_iter). En el peor caso, la complejidad sería O(max_iter). Dentro del bucle, se realizan operaciones como generar nodos aleatorios, encontrar el nodo más cercano, realizar la dirección (steering), verificar si hay obstáculos, calcular la distancia, etc., todas estas operaciones tienen una complejidad de tiempo constante (O(1)).
#
# RRTStar.random_node(): O(1) (operación constante).
#
# RRTStar.nearest_node(): O(n) en el peor caso, donde n es el número de nodos en la lista de nodos (self.node_list). Esto se debe a que se calcula la distancia entre el nodo dado y todos los nodos existentes en la lista y se encuentra el más cercano.
#
# RRTStar.steer(): O(1) (operación constante).
#
# RRTStar.distance(): O(1) (operación constante).
#
# RRTStar.obstacle_free(): El costo computacional de esta función depende de la longitud de la línea entre los dos nodos (from_node y to_node). Si la longitud de la línea es L, el costo será proporcional a L. En el peor caso, cuando la línea cruza una gran cantidad de obstáculos, el costo puede ser O(L).
#
# RRTStar.near_nodes(): O(n) en el peor caso, donde n es el número de nodos en la lista de nodos (self.node_list). Esto se debe a que se calcula la distancia entre el nodo dado y todos los nodos existentes en la lista para encontrar los nodos cercanos dentro de un radio determinado.
#
# RRTStar.cost_to_node(): O(d), donde d es la profundidad del árbol de nodos hasta el nodo dado. En el peor caso, si todos los nodos están conectados linealmente, el costo puede ser proporcional al número de nodos en el camino.
#
# RRTStar.rewire(): O(n) en el peor caso, donde n es la cantidad de nodos cercanos proporcionados. Esto se debe a que se comprueba si es beneficioso volver a enlazar cada nodo cercano y se realiza una verificación de obstáculos.
#
# RRTStar.extract_path(): O(d), donde d es la profundidad del árbol de nodos desde el nodo final hasta el nodo inicial. Esto se debe a que se sigue el camino desde el nodo final hasta el nodo inicial y se guarda en una lista, que luego se invierte.