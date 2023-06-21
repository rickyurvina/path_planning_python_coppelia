import numpy as np
import matplotlib.pyplot as plt
from numpy.distutils.fcompiler import none
from src.components.loadData import load_data_occupancy_grid
from src.components.loadData import load_solution_data


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



def main_rrt(occupancy_grid=none, ordered_positions=none):
    try:
        occupancy_grid = load_data_occupancy_grid()['occupancy_grid']
        ordered_positions = load_solution_data()['ordered_positions']
        print(ordered_positions)
        start = (210, 50)
        goals = [(ordered_positions[0][0]+203.8, ordered_positions[0][1]+120.705)]


        # Crear el planificador RRT*
        rrt_star = RRTStar(start, goals[0], occupancy_grid)

        # Generar el camino
        path = rrt_star.generate_path()

        # Visualizar el occupancy grid y el camino
        plt.imshow(occupancy_grid, cmap='gray', origin='lower')
        plt.plot(start[0], start[1], 'ro', label='Start')
        plt.plot(goals[0][0], goals[0][1], 'go', label='Goal')
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

main_rrt()
