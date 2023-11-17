import numpy as np

from src.components.step_3_rrt.distance import distance


def get_new_point_in_ellipsoid(self, goal_bias, c_best, start, goal):
    if np.random.random() < goal_bias:
        point = [goal.row, goal.col]
    else:
        c_min = distance(self, start, goal)
        x_start = np.array([start.row, start.col]).reshape((2, 1))
        x_goal = np.array([goal.row, goal.col]).reshape((2, 1))
        x_center = (x_start + x_goal) / 2
        a1 = (x_goal - x_start) / c_min
        I1 = np.array([[1, 0]])
        M = a1.dot(I1)
        U, S, Vt = np.linalg.svd(M)
        D = np.diag([1, np.linalg.det(U) * np.linalg.det(Vt)])
        C = (U.dot(D)).dot(Vt)
        L = np.diag([c_best / 2, np.sqrt(c_best ** 2 - c_min ** 2) / 2])
        r = np.random.random()
        theta = np.random.uniform(-np.pi, np.pi)
        x_ball = [[r * np.cos(theta)], [r * np.sin(theta)]]
        x_rand = (C.dot(L)).dot(x_ball) + x_center
        point = [int(x_rand[0]), int(x_rand[1])]
    return point
