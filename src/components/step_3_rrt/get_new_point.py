import numpy as np

from src.steps import config


def get_new_point(self, goal_bias, start, goal):
    min_x = min(start.col, goal.col) - config.LIMIT_AOI
    max_x = max(start.col, goal.col) + config.LIMIT_AOI
    min_y = min(start.row, goal.row) - config.LIMIT_AOI
    max_y = max(start.row, goal.row) + config.LIMIT_AOI
    middle_y = 500
    min_x = max(min_x, 0)
    max_x = min(max_x, 999)
    if min_y <= middle_y and max_y <= middle_y:
        min_y = int(0)
        max_y = 1000
    else:
        min_y = 0
        max_y = 1000

    self.size_x_min = min_x
    self.size_x_max = max_x
    self.size_y_min = min_y
    self.size_y_max = max_y
    if np.random.random() < goal_bias:
        point = [goal.row, goal.col]
    else:
        point = [np.random.randint(min_x, self.size_x_max - 1), np.random.randint(min_y, 1000 - 1)]
    return point
