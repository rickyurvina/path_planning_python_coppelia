import numpy as np

from src.components.step_3_rrt.check_collision import check_collision
from src.components.step_3_rrt.distance import distance
from src.components.step_3_rrt.get_nearest_node import get_nearest_node
from src.components.step_3_rrt.informed_rrt import Node


def extend_node(self, goal, new_point, extend_dis=300):
    nearest_node = get_nearest_node(new_point)
    slope = np.arctan2(new_point[1] - nearest_node.col, new_point[0] - nearest_node.row)
    new_row = nearest_node.row + extend_dis * np.cos(slope)
    new_col = nearest_node.col + extend_dis * np.sin(slope)
    new_node = Node(int(new_row), int(new_col))
    if (self.size_y_min <= new_row < self.size_y_max) and (self.size_x_min <= new_col < self.size_x_max) and \
            not check_collision(self, nearest_node, new_node):
        new_node.parent = nearest_node
        new_node.cost = extend_dis
        self.vertices.append(new_node)
        if not self.found:
            d = distance(self, new_node, goal)
            if d < extend_dis:
                goal.cost = d
                goal.parent = new_node
                self.vertices.append(goal)
                self.found = True
        return new_node
    else:
        return None
