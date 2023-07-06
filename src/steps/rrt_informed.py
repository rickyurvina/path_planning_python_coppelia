import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
from src.components import loadFiles, saveFiles
from dotenv import load_dotenv
import os

load_dotenv()


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node / edge
        self.cost = 0.0  # cost to parent / edge weight


# Class for RRT
class RRT_INFORMED:
    # Constructor
    def __init__(self, map_array, goals, name_folder):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size
        self.goals = [Node(goal[1], goal[0]) for goal in goals]  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag
        self.name_folder = name_folder

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.goals[0])

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        return np.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)

    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                             np.linspace(node1.col, node2.col, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_point(self, goal_bias, goal):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # select goal
        if np.random.random() < goal_bias:
            point = [goal.row, goal.col]
        # or generate a random point
        else:
            point = [np.random.randint(0, self.size_row - 1), np.random.randint(0, self.size_col - 1)]
        return point

    def get_new_point_in_ellipsoid(self, goal_bias, c_best, start, goal):

        if np.random.random() < goal_bias:
            point = [goal.row, goal.col]

        #### TODO ####
        # Generate a random point in an ellipsoid
        else:
            # pass
            # Compute the distance between start and goal - c_min
            c_min = self.dis(start, goal)
            # Calculate center of the ellipsoid - x_center
            x_start = np.array([start.row, start.col]).reshape((2, 1))
            x_goal = np.array([goal.row, goal.col]).reshape((2, 1))
            x_center = (x_start + x_goal) / 2
            # Compute rotation matrix from elipse to world frame - C
            a1 = (x_goal - x_start) / c_min
            I1 = np.array([[1, 0]])
            M = a1.dot(I1)
            U, S, Vt = np.linalg.svd(M)
            D = np.diag([1, np.linalg.det(U) * np.linalg.det(Vt)])
            C = (U.dot(D)).dot(Vt)
            # Compute diagonal matrix - L
            L = np.diag([c_best / 2, np.sqrt(c_best ** 2 - c_min ** 2) / 2])
            r = np.random.random()
            theta = np.random.uniform(-np.pi, np.pi)
            x_ball = [[r * np.cos(theta)], [r * np.sin(theta)]]
            x_rand = (C.dot(L)).dot(x_ball) + x_center

            point = [int(x_rand[0]), int(x_rand[1])]

        #### TODO END ####

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
        #### TODO END ####

        return new_point

    def extend(self, goal, new_point, extend_dis=10):

        nearest_node = self.get_nearest_node(new_point)

        # Calculate new node location
        slope = np.arctan2(new_point[1] - nearest_node.col, new_point[0] - nearest_node.row)
        new_row = nearest_node.row + extend_dis * np.cos(slope)
        new_col = nearest_node.col + extend_dis * np.sin(slope)
        new_node = Node(int(new_row), int(new_col))

        # Check boundary and collision
        if (0 <= new_row < self.size_row) and (0 <= new_col < self.size_col) and \
                not self.check_collision(nearest_node, new_node):
            # If pass, add the new node
            new_node.parent = nearest_node
            new_node.cost = extend_dis
            self.vertices.append(new_node)

            # Check if goal is close
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

    def path_cost(self, start_node, end_node):

        cost = 0
        curr_node = end_node
        while start_node.row != curr_node.row or start_node.col != curr_node.col:
            # Keep tracing back until finding the start_node 
            # or no path exists
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent

        return cost

    def rewire(self, new_node, neighbors, start):

        if neighbors == []:
            return

        # Compute the distance from the new node to the neighbor nodes
        distances = [self.dis(new_node, node) for node in neighbors]

        costs = [d + self.path_cost(start, neighbors[i]) for i, d in enumerate(distances)]
        indices = np.argsort(np.array(costs))
        # check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break

        for i, node in enumerate(neighbors):
            # new cost
            new_cost = self.path_cost(start, new_node) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(start, node) > new_cost and \
                    not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]

    def draw_map(self, path, name='RRT'):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        # ax.imshow(self.map_array, origin='lower')
        ax.imshow(self.map_array, cmap='gray', origin='lower')

        # Draw Final Path if found
        if self.found:
            for index, (goal) in enumerate(path):
                cur = goal
                start = self.goals[index]
                while cur.col != start.col or cur.row != start.row:
                    plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                    cur = cur.parent
                # plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        plt.plot(self.goals[0].col, self.goals[0].row, markersize=5, marker='o', color='g')
        plt.plot(goal.col, goal.row, markersize=5, marker='o', color='r')
        plt.title(name)
        plt.xlabel('X')
        plt.ylabel('Y')
        filename = saveFiles.get_name_to_save_plot(self.name_folder, 'rrt')
        plt.savefig(filename, dpi=500)

        plt.show()

    def informed_RRT_star(self, n_pts=1000, neighbor_size=20):
        path = []
        search_vertices = []

        self.init_map()
        # Start searching
        for index, (position) in enumerate(self.goals):
            print("position index", index)
            start = position
            goal = self.goals[index + 1]
            if int(os.getenv("BREAK_AT")) == 10:
                break
            if index > 0:
                self.found = False
                self.vertices = []
                self.vertices.append(start)
                # break

            for i in range(n_pts):
                c_best = 0

                if self.found:
                    c_best = self.path_cost(start, goal)

                new_point = self.sample(start, goal, 0.05, c_best)
                new_node = self.extend(goal, new_point, 10)
                if new_node is not None:
                    neighbors = self.get_neighbors(new_node, neighbor_size)
                    self.rewire(new_node, neighbors, start)

            if self.found:
                steps = len(self.vertices) - 2
                length = self.path_cost(start, goal)
                print("It took %d nodes to find the current path" % steps)
                print("The path length is %.2f" % length)
            else:
                print("No path found for RRT informed")
            path.append(goal)
            search_vertices.append(self.vertices)

        self.draw_map(path, "RRT INFORMED")
        return path
