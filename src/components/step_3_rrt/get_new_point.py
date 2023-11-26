import numpy as np

from src.steps import config


def get_new_point(self, goal_bias, start, goal):
    min_x = min(start.col, goal.col) - config.LIMIT_AOI
    max_x = max(start.col, goal.col) + config.LIMIT_AOI
    min_y = min(start.row, goal.row) - config.LIMIT_AOI
    max_y = max(start.row, goal.row) + config.LIMIT_AOI
    middle_y = config.RESOLUTION_Y / 2
    min_x = max(min_x, 0)
    max_x = min(max_x, config.RESOLUTION_Y - 1)
    max_y_search = config.RESOLUTION_Y
    if min_y <= middle_y and max_y <= middle_y:
        min_y = int(0)
        max_y = middle_y + config.LIMIT_AOI
        max_y_search = middle_y + config.LIMIT_AOI
    else:
        min_y = 0
        max_y = config.RESOLUTION_Y

    if self.name_method == 'IRRT':
        self.size_x_min = min_x
        self.size_x_max = max_x
        self.size_y_min = min_y
        self.size_y_max = max_y
    else:
        self.size_x_min = 0
        self.size_x_max = config.RESOLUTION_X
        self.size_y_min = 0
        self.size_y_max = config.RESOLUTION_Y

    if np.random.random() < goal_bias:
        point = [goal.row, goal.col]
    else:
        # if self.name_method == 'IRRT':
        #     point = [np.random.randint(min_x, self.size_x_max - 1), np.random.randint(min_y, max_y_search)]
        # else:
        point = [np.random.randint(self.size_y_min, self.size_y_max - 1),
                 np.random.randint(self.size_x_min, self.size_x_max - 1)]
    return point
