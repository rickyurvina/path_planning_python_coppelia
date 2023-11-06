import numpy as np
from scipy import spatial
import traceback
from colorama import init, Fore
from src.components import get_row_of_position
import matplotlib.pyplot as plt
from src.components import save_files
from src.steps import config

init()


class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node / edge
        self.cost = 0.0  # cost to parent / edge weight
        self.target_speed = 10.0 / 3.6
        self.yaw_th = np.deg2rad(0.3)
        self.xy_th = 0.5
        self.invalid_travel_ratio = 5.0
        self.robot_radius = 0.0
        self.connect_circle_dist = 50.0


class RRT:
    def __init__(self, map_array, goals, rows, ordered_positions, name_folder):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.rows = rows  # map array, 1->free, 0->obstacle
        self.ordered_positions = ordered_positions  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size
        self.goals = [Node(goal[1], goal[0]) for goal in goals]  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag
        self.name_folder = name_folder

    def init_map(self):
        self.found = False
        self.vertices = []
        self.vertices.append(self.goals[0])

    def dis(self, node1, node2):
        return np.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)

    def check_collision(self, node1, node2):
        if node2 is None:
            return False
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                             np.linspace(node1.col, node2.col, dtype=int))
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_point(self, goal_bias, goal):
        if np.random.random() < goal_bias:
            point = [goal.row, goal.col]
        else:
            point = [np.random.randint(0, self.size_row - 1), np.random.randint(0, self.size_col - 1)]
        return point

    def get_new_point_in_ellipsoid(self, goal_bias, c_best, start, goal):
        if np.random.random() < goal_bias:
            point = [goal.row, goal.col]
        else:
            c_min = self.dis(start, goal)
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

    def get_nearest_node(self, point):
        samples = [[v.row, v.col] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        coord, ind = kdtree.query(point)
        return self.vertices[ind]

    def sample(self, start, goal, goal_bias=0.05, c_best=0):
        if c_best <= 0:
            new_point = self.get_new_point(goal_bias, goal)
        else:
            new_point = self.get_new_point_in_ellipsoid(goal_bias, c_best, start, goal)
        return new_point

    def extend(self, goal, new_point, extend_dis=10):
        nearest_node = self.get_nearest_node(new_point)
        slope = np.arctan2(new_point[1] - nearest_node.col, new_point[0] - nearest_node.row)
        new_row = nearest_node.row + extend_dis * np.cos(slope)
        new_col = nearest_node.col + extend_dis * np.sin(slope)
        new_node = Node(int(new_row), int(new_col))
        if (0 <= new_row < self.size_row) and (0 <= new_col < self.size_col) and \
                not self.check_collision(nearest_node, new_node):
            new_node.parent = nearest_node
            new_node.cost = extend_dis
            self.vertices.append(new_node)
            if not self.found:
                d = self.dis(new_node, goal)
                if d < extend_dis:
                    goal.cost = d
                    goal.parent = new_node
                    self.vertices.append(goal)
                    self.found = True
            return new_node
        else:
            return None

    def get_neighbors(self, new_node, neighbor_size):
        samples = [[v.row, v.col] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        ind = kdtree.query_ball_point([new_node.row, new_node.col], neighbor_size)
        neighbors = [self.vertices[i] for i in ind]
        neighbors.remove(new_node)
        return neighbors

    def path_cost(self, start_node, end_node, old_cost=0):
        cost = 0
        curr_node = end_node
        npi = 0
        while round(start_node.row, 3) != round(curr_node.row, 3) or round(start_node.col, 3) != round(curr_node.col,
                                                                                                       3):
            npi = npi + 1
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
            if old_cost > cost:
                return old_cost

        return cost

    def rewire(self, new_node, neighbors, start):

        if neighbors == []:
            return
        distances = [self.dis(new_node, node) for node in neighbors]
        costs = [d + self.path_cost(start, neighbors[i]) for i, d in enumerate(distances)]
        indices = np.argsort(np.array(costs))

        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break
        for i, node in enumerate(neighbors):
            old_cost = costs[i] - distances[i]
            new_cost = self.path_cost(start, new_node, old_cost) + distances[i]
            if old_cost > new_cost and \
                    not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]

    def draw_map(self, path, name='RRT'):
        try:
            fig, ax = plt.subplots(1)
            ax.imshow(self.map_array, cmap='gray', origin='lower')

            if self.found:
                for index, (goal) in enumerate(path):
                    cur = goal
                    if cur.parent is not None:
                        start = self.goals[index]
                        # Draw Trees or Sample points
                        # for node in self.vertices[1:-1]:
                        #     plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
                        #     plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
                        lines = []
                        while cur.col != start.col or cur.row != start.row:
                            lines.append([cur.col, cur.row, cur.parent.col, cur.parent.row])
                            cur = cur.parent
                        lines = np.array(lines)
                        plt.plot(lines[:, 0], lines[:, 1], lines[:, 2], lines[:, 3], color='b')

            plt.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g', label='Start')
            if goal:
                plt.plot(goal.col, goal.row, markersize=5, marker='o', color='r', label='Goal')
            plt.title(name)
            plt.legend()
            plt.xlabel('X')
            plt.ylabel('Y')
            filename = save_files.get_name_to_save_plot(self.name_folder, 'rrt')
            plt.savefig(filename, dpi=500)

            plt.show()
        except Exception as e:
            print(Fore.RED + e)
            traceback.print_exc()

    def informed_RRT_star(self, n_pts=config.MIN_ITER):
        try:
            path = []
            search_vertices = []
            sum_path_length = 0
            self.init_map()
            for index, (position) in enumerate(self.goals):
                if index == config.BREAK_AT:
                    break
                print(config.POSITION_INDEX, index)
                if index != len(self.goals) - 1:
                    start = position
                    goal = self.goals[index + 1]
                    if index > 0:
                        self.found = False
                        self.vertices = []
                        self.vertices.append(start)
                        actual_row = get_row_of_position.find_row_for_position(self.rows, self.ordered_positions[index])
                        next_row = get_row_of_position.find_row_for_position(self.rows,
                                                                             self.ordered_positions[index + 1])
                        if actual_row != next_row:
                            n_pts = config.MAX_ITER
                        else:
                            n_pts = config.MIN_ITER

                    for i in range(n_pts):
                        c_best = 0
                        if self.found:
                            c_best = self.path_cost(start, goal, c_best)
                        new_point = self.sample(start, goal, config.GOAL_SAMPLE_RATE, c_best)
                        new_node = self.extend(goal, new_point, config.RADIUS)
                        if new_node is not None:
                            neighbors = self.get_neighbors(new_node, config.NEIGHBOR_SIZE)
                            self.rewire(new_node, neighbors, start)
                    if self.found:
                        steps = len(self.vertices) - 2
                        print(config.MESSAGE_PATH % steps)
                    else:
                        print(Fore.RED + config.PATH_NO_FOUND)
                    path.append(goal)
                    search_vertices.append(self.vertices)
            print(config.MESSAGE_DONE)
            self.draw_map(path, config.METHOD)
            print(config.MESSAGE_PLOTTED)
            return path, sum_path_length
        except Exception as e:
            print(Fore.RED + e)
            traceback.print_exc()
