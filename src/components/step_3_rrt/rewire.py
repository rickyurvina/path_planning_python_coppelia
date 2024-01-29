import numpy as np

from src.components.step_3_rrt.check_collision import check_collision
from src.components.step_3_rrt.check_collision_with_clearance import check_collision_with_clearance
from src.components.step_3_rrt.distance import distance
from src.components.step_3_rrt.path_cost import path_cost


def rewire(self, new_node, neighbors, start):
    if neighbors == []:
        return
    distances = [distance(new_node, node) for node in neighbors]
    costs = [d + path_cost(self, start, neighbors[i]) for i, d in enumerate(distances)]
    indices = np.argsort(np.array(costs))

    if self.name_method == 'IRRT':

        for i in indices:

            if not check_collision_with_clearance(self, new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break
        for i, node in enumerate(neighbors):

            old_cost = costs[i] - distances[i]
            new_cost = path_cost(self, start, new_node, old_cost) + distances[i]
            if old_cost > new_cost and \
                    not check_collision_with_clearance(self, node, new_node):
                node.parent = new_node
                node.cost = distances[i]
    else:

        for i in indices:
            collision, _ = check_collision(self, new_node, neighbors[i])
            if not collision:
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break
        for i, node in enumerate(neighbors):
            old_cost = costs[i] - distances[i]
            new_cost = path_cost(self, start, new_node, old_cost) + distances[i]
            collision, obstacle_distance = check_collision(self, node, new_node)

            if old_cost > new_cost and \
                    not collision:
                self.distances_obstacles.append(obstacle_distance)
                node.parent = new_node
                node.cost = distances[i]
