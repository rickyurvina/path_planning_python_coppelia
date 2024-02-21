import numpy as np
import traceback
from colorama import init, Fore
from src.components.step_2_tsp import get_row_of_position
from src.components.step_3_rrt.check_collision import check_collision
from src.components.step_3_rrt.check_collision_with_clearance import check_collision_with_clearance
from src.components.step_3_rrt.distance import distance
from src.components.step_3_rrt.draw_map_rrt import draw_map, draw_combined_maps
from src.components.step_3_rrt.find_nearest_obstacle import find_nearest_obstacle
from src.components.step_3_rrt.get_nearest_node import get_nearest_node
from src.components.step_3_rrt.get_neighbors import get_neighbors
from src.components.step_3_rrt.path_cost import path_cost
from src.components.step_3_rrt.path_cost_final import path_cost_final
from src.components.step_3_rrt.path_cost_final_smooth import path_cost_final_smoothed
from src.components.step_3_rrt.path_to_soomth_path import to_smooth_path
from src.components.step_3_rrt.rewire import rewire
from src.components.step_3_rrt.sample import sample
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files, save_files
from src.components import create_folder
from src.components.step_3_rrt.unicycle_robot import UnicycleRobot
from src.steps import config
import time
from src.components.step_3_rrt.path_curvature import calculate_curvature, calculate_curvature_variation, plot_curvature

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
        self.model = UnicycleRobot(0.1, 0.5, 0.1, 1.0, 0.5)
        self.total_nodes = 0
        self.execution_time_rrt = 0


class RRT:
    def __init__(self, map_array, map_array_rgb, goals, rows, ordered_positions, name_folder,
                 path_solutions=config.PATH_FOLDER):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.map_array_rgb = map_array_rgb  # map array, 1->free, 0->obstacle
        self.rows = rows  # map array, 1->free, 0->obstacle
        self.ordered_positions = ordered_positions  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size
        self.size_x_min = 0
        self.size_x_max = self.size_row
        self.size_y_min = 0
        self.size_y_max = self.size_col
        self.goals = [Node(goal[1], goal[0]) for goal in goals]  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag
        self.name_folder = name_folder
        self.path_solutions = path_solutions
        self.model = UnicycleRobot(0.1, 0.5, 0.1, 1.0, 0.5)
        self.total_collisions = 0
        self.total_planning_time = 0
        self.total_cost = 0
        self.font_size = 18
        self.total_samples = 0
        self.start_time = 0
        self.success = True
        self.path = []
        self.search_vertices = []
        self.name_method = "IRRT"
        self.smoot_path = []
        self.distances_obstacles = []
        self.curvature = 0
        self.smoothness = 0

    def init_map(self):
        self.found = False
        self.vertices = []
        self.vertices.append(self.goals[0])
        self.total_nodes = 0
        self.total_cost = 0
        self.total_collisions = 0
        self.total_planning_time = 0
        self.total_samples = 0
        self.start_time = time.time()
        self.success = True
        self.path = []
        self.search_vertices = []
        self.name_method = ""
        self.smoot_path = []
        self.distances_obstacles = []
        self.curvature = 0
        self.smoothness = 0

    def extend(self, goal, new_point, extend_dis=10):
        nearest_node = get_nearest_node(self, new_point)
        slope = np.arctan2(new_point[1] - nearest_node.col, new_point[0] - nearest_node.row)
        new_row = nearest_node.row + extend_dis * np.cos(slope)
        new_col = nearest_node.col + extend_dis * np.sin(slope)
        new_node = Node(int(new_row), int(new_col))
        if self.name_method == 'IRRT':
            if (self.size_y_min <= new_row < self.size_y_max) and (
                    self.size_x_min <= new_col < self.size_x_max) and not check_collision_with_clearance(self,
                                                                                                         nearest_node,
                                                                                                         new_node):
                new_node.parent = nearest_node
                new_node.cost = extend_dis
                self.vertices.append(new_node)
                if not self.found:
                    d = distance(new_node, goal)
                    if d < extend_dis:
                        goal.cost = d
                        goal.parent = new_node
                        self.vertices.append(goal)
                        self.found = True
                return new_node
            else:
                return None
        else:
            if (self.size_y_min <= new_row < self.size_y_max) and (
                    self.size_x_min <= new_col < self.size_x_max) and not check_collision(self, nearest_node, new_node):
                new_node.parent = nearest_node
                new_node.cost = extend_dis
                self.vertices.append(new_node)
                if not self.found:
                    d = distance(new_node, goal)
                    nearest_obstacle, obstacle_distance = find_nearest_obstacle(self.map_array, new_node)
                    self.distances_obstacles.append(
                        obstacle_distance) if (self.name_method == 'RRT' and obstacle_distance != 0) else None
                    if d < extend_dis:
                        goal.cost = d
                        goal.parent = new_node
                        self.vertices.append(goal)
                        self.found = True
                return new_node
            else:
                return None

    def rrt(self, n_pts=config.MIN_ITER):
        try:
            path = []
            search_vertices = []
            self.init_map()
            self.name_method = "RRT"
            for index, (position) in enumerate(self.goals):

                if index == config.BREAK_AT:
                    break
                print(config.POSITION_INDEX, index)
                if index != len(self.goals) - 1:
                    start = position
                    goal = self.goals[index + 1]
                    self.found = False
                    self.vertices = []
                    self.vertices.append(start)
                    n_pts = config.MAX_ITER
                    for i in range(n_pts):
                        if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                            print(Fore.RED + "Time is up")
                            return None
                        new_point = sample(self, start, goal, config.GOAL_SAMPLE_RATE)
                        new_node = self.extend(goal, new_point, config.RADIUS)
                        if self.found:
                            break
                    if self.found:
                        steps = len(self.vertices) - 2
                        self.total_nodes += steps
                        self.total_cost += path_cost_final(self, start, goal)
                        print(config.MESSAGE_PATH % steps)
                        print(self.distances_obstacles)
                        print("Promedio de distancias", np.mean(self.distances_obstacles))
                        print("Desviación estándar de distancias", np.std(self.distances_obstacles))
                        print("Máxima distancia", np.max(self.distances_obstacles))
                        print("Mínima distancia", np.min(self.distances_obstacles))
                        print("Varianza de distancias", np.var(self.distances_obstacles))

                    else:
                        print(Fore.RED + config.PATH_NO_FOUND)
                        self.found = True
                        break

                    path.append(goal)
                    search_vertices.append(self.vertices)

            if self.found:
                print(config.MESSAGE_DONE)
                end_time = time.time()
                self.total_planning_time = end_time - self.start_time
                self.path = path
                self.name_method = "RRT"
                self.search_vertices = search_vertices
                res_curvature, self.smoothness = calculate_curvature_variation(self)
                self.curvature = np.mean(res_curvature)
                plot_curvature(res_curvature)
                print("promedio de la curvatura", np.mean(self.curvature))
                print("Suavidad", self.smoothness)
                if config.NO_PLOT_RRT:
                    draw_map(self)
                    # draw_combined_maps(self, path, "RRT")
                    print(config.MESSAGE_PLOTTED)
                return self
            return None
        except Exception as e:
            print(Fore.RED + str(e))
            traceback.print_exc()

    def rrt_star(self, n_pts=1000):
        try:
            path = []
            search_vertices = []
            self.init_map()

            for index, (position) in enumerate(self.goals):
                if index == config.BREAK_AT:
                    break
                print(config.POSITION_INDEX, index)
                if index != len(self.goals) - 1:
                    start = position
                    goal = self.goals[index + 1]
                    self.found = False
                    self.vertices = []
                    self.vertices.append(start)
                    n_pts = config.MAX_ITER

                    for i in range(n_pts):
                        if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                            print(Fore.RED + "Time is up")
                            return None
                        # Extend a new node
                        new_point = sample(self, start, goal, config.GOAL_SAMPLE_RATE)
                        new_node = self.extend(goal, new_point, config.RADIUS)
                        # Rewire
                        if new_node is not None:
                            neighbors = get_neighbors(self, new_node, config.NEIGHBOR_SIZE)
                            rewire(self, new_node, neighbors, start)
                    if self.found:
                        steps = len(self.vertices) - 2
                        self.total_nodes += steps
                        self.total_cost += path_cost_final(self, start, goal)
                        print(config.MESSAGE_PATH % steps)
                    else:
                        print(Fore.RED + config.PATH_NO_FOUND)
                        self.found = True
                        break

                    path.append(goal)
                    search_vertices.append(self.vertices)

            # Output
            if self.found:
                print(config.MESSAGE_DONE)
                end_time = time.time()
                self.total_planning_time = end_time - self.start_time
                self.path = path
                self.name_method = "RRT-Star"
                self.search_vertices = search_vertices
                curvature, self.smoothness = calculate_curvature_variation(self)
                self.curvature = np.mean(curvature)
                # print("curvature", curvature)
                plot_curvature(curvature)
                print("promedio de la curvatura", np.mean(self.curvature))
                print("Suavidad", self.smoothness)
                if config.NO_PLOT_RRT:
                    draw_map(self)
                    # draw_combined_maps(self, path, "RRT")
                    print(config.MESSAGE_PLOTTED)
                    print(self.distances_obstacles)
                    print("Promedio de distancias", np.mean(self.distances_obstacles))
                    print("Desviación estándar de distancias", np.std(self.distances_obstacles))
                    print("Máxima distancia", np.max(self.distances_obstacles))
                    print("Mínima distancia", np.min(self.distances_obstacles))
                    print("Varianza de distancias", np.var(self.distances_obstacles))

                return self
            return None
        except Exception as e:
            print(Fore.RED + str(e))
            traceback.print_exc()

    def rrt_informed(self, n_pts=config.MIN_ITER):
        try:
            path = []
            search_vertices = []
            self.init_map()
            self.name_method = "IRRT"
            for index, (position) in enumerate(self.goals):
                if index == config.BREAK_AT:
                    break
                print(config.POSITION_INDEX, index)
                if index != len(self.goals) - 1:
                    start = position
                    goal = self.goals[index + 1]
                    self.found = False
                    self.vertices = []
                    self.vertices.append(start)
                    n_pts = config.MAX_ITER
                    for i in range(n_pts):
                        if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                            print(Fore.RED + "Time is up")
                            return None
                        c_best = 0
                        if self.found:
                            c_best = path_cost(self, start, goal, c_best)
                        new_point = sample(self, start, goal, config.GOAL_SAMPLE_RATE, c_best)
                        new_node = self.extend(goal, new_point, config.RADIUS)
                        if new_node is not None:
                            neighbors = get_neighbors(self, new_node, config.NEIGHBOR_SIZE)
                            rewire(self, new_node, neighbors, start)
                    if self.found:
                        steps = len(self.vertices) - 2
                        self.total_nodes += steps
                        self.total_cost += path_cost_final(self, start, goal)
                        print(config.MESSAGE_PATH % steps)
                    else:
                        print(Fore.RED + config.PATH_NO_FOUND)
                        self.found = True
                        break
                    path.append(goal)
                    search_vertices.append(self.vertices)
            if self.found:
                print(config.MESSAGE_DONE)
                end_time = time.time()
                self.total_planning_time = end_time - self.start_time
                self.path = path
                self.search_vertices = search_vertices
                self.smoot_path = np.array(to_smooth_path(self))
                self.path = self.smoot_path
                print(config.MESSAGE_PATH % self.total_cost)
                print(self.distances_obstacles)
                print("Promedio de distancias", np.mean(self.distances_obstacles))
                print("Desviación estándar de distancias", np.std(self.distances_obstacles))
                print("Máxima distancia", np.max(self.distances_obstacles))
                print("Mínima distancia", np.min(self.distances_obstacles))
                print("Varianza de distancias", np.var(self.distances_obstacles))
                curvature, self.smoothness = calculate_curvature_variation(self)
                self.curvature = np.mean(curvature)
                # print("curvature", curvature)
                plot_curvature(curvature)
                print("promedio de la curvatura", np.mean(self.curvature))
                print("Suavidad", self.smoothness)

                if config.NO_PLOT_RRT:
                    draw_map(self)
                    # draw_combined_maps(self, path, "RRT")
                    print(config.MESSAGE_PLOTTED)

                return self
            return None
        except Exception as e:
            print(Fore.RED + str(e))
            traceback.print_exc()


def main():
    print("Start informed RRT Unicycle star planning")
    data = load_files.load_solution_data()
    ordered_transformed = shift_positions(data['ordered_positions'])
    name_folder = create_folder.create_folder("../../solutions")
    path_solutions = "../../solutions"
    RRT_PLANNER = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'], data['ordered_positions'],
                      name_folder, path_solutions)

    # RRT_PLANNER.rrt()
    # RRT_PLANNER.rrt_star()
    RRT_PLANNER.rrt_informed()


if __name__ == '__main__':
    main()
