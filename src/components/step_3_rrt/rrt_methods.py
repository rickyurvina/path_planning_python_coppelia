import traceback
from colorama import init, Fore
from src.components.step_2_tsp import get_row_of_position
from src.components.step_3_rrt.draw_map_rrt import draw_map, draw_combined_maps
from src.steps import config
import time

init()


def rrt(self, n_pts=config.MIN_ITER):
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
                if index > 0:
                    self.found = False
                    self.vertices = []
                    self.vertices.append(start)
                    actual_row = get_row_of_position.find_row_for_position(self.rows,
                                                                           self.ordered_positions[index])
                    next_row = get_row_of_position.find_row_for_position(self.rows,
                                                                         self.ordered_positions[index + 1])
                    if actual_row != next_row:
                        n_pts = config.MAX_ITER
                    else:
                        n_pts = config.MIN_ITER
                i = 0

                for i in range(n_pts):
                    if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                        print(Fore.RED + "Time is up")
                        return None, None, None, None, None, None, None, False
                    new_point = self.sample(start, goal, config.GOAL_SAMPLE_RATE)
                    new_node = self.extend(goal, new_point, config.RADIUS)
                    if self.found:
                        break
                if self.found:
                    steps = len(self.vertices) - 2
                    self.total_nodes += steps
                    self.total_cost += self.path_cost_final(start, goal)
                    print(config.MESSAGE_PATH % steps)
                else:
                    print(Fore.RED + config.PATH_NO_FOUND)
                path.append(goal)
                search_vertices.append(self.vertices)

                # Output
        if self.found:
            print(config.MESSAGE_DONE)
            end_time = time.time()
            self.total_planning_time = end_time - self.start_time
            draw_map(self, path, "RRT")
            draw_combined_maps(self, path, "RRT")
            print(config.MESSAGE_PLOTTED)
            return "RRT", self.total_nodes, self.total_cost, self.total_collisions, self.total_planning_time, search_vertices, self.total_samples, self.success
        return None, None, None, None, None, None, None, False
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
                if index > 0:
                    self.found = False
                    self.vertices = []
                    self.vertices.append(start)
                    actual_row = get_row_of_position.find_row_for_position(self.rows,
                                                                           self.ordered_positions[index])
                    next_row = get_row_of_position.find_row_for_position(self.rows,
                                                                         self.ordered_positions[index + 1])
                    if actual_row != next_row:
                        n_pts = config.MAX_ITER
                    else:
                        n_pts = config.MIN_ITER
                i = 0
                for i in range(n_pts):
                    if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                        print(Fore.RED + "Time is up")
                        return None, None, None, None, None, None, None, False
                    # Extend a new node
                    new_point = self.sample(start, goal, config.GOAL_SAMPLE_RATE)
                    new_node = self.extend(goal, new_point, config.RADIUS)
                    # Rewire
                    if new_node is not None:
                        neighbors = self.get_neighbors(new_node, config.NEIGHBOR_SIZE)
                        self.rewire(new_node, neighbors, start)
                if self.found:
                    steps = len(self.vertices) - 2
                    self.total_nodes += steps
                    self.total_cost += self.path_cost_final(start, goal)
                    print(config.MESSAGE_PATH % steps)
                else:
                    print(Fore.RED + config.PATH_NO_FOUND)
                path.append(goal)
                search_vertices.append(self.vertices)

        # Output
        if self.found:
            print(config.MESSAGE_DONE)
            end_time = time.time()
            self.total_planning_time = end_time - self.start_time
            draw_map(self, path, "RRT_Star")
            draw_combined_maps(self, path, "RRT_Star")
            print(config.MESSAGE_PLOTTED)
            return "RRT-Star", self.total_nodes, self.total_cost, self.total_collisions, self.total_planning_time, search_vertices, self.total_samples, self.success

        return None, None, None, None, None, None, None, False

    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def rrt_informed(self, n_pts=config.MIN_ITER):
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
                i = 0

                for i in range(n_pts):
                    if (time.time() >= (self.start_time + config.TIME_LIMIT)):
                        print(Fore.RED + "Time is up")
                        return None, None, None, None, None, None, None, False
                    c_best = 0
                    if self.found:
                        c_best = self.path_cost(start, goal, c_best)
                    new_point = self.sample(start, goal, config.GOAL_SAMPLE_RATE, c_best)
                    new_node = self.extend(goal, new_point, config.RADIUS)
                    # new_node = self.extend_unicycle(goal, new_point, config.RADIUS)
                    if new_node is not None:
                        neighbors = self.get_neighbors(new_node, config.NEIGHBOR_SIZE)
                        self.rewire(new_node, neighbors, start)
                if self.found:
                    steps = len(self.vertices) - 2
                    self.total_nodes += steps
                    self.total_cost += self.path_cost_final(start, goal)
                    print(config.MESSAGE_PATH % steps)
                else:
                    print(Fore.RED + config.PATH_NO_FOUND)
                path.append(goal)
                search_vertices.append(self.vertices)
        if self.found:
            print(config.MESSAGE_DONE)
            end_time = time.time()
            self.total_planning_time = end_time - self.start_time
            draw_map(self, path, "RRT_Informed")
            draw_combined_maps(self, path, "RRT_Informed")
            print(config.MESSAGE_PLOTTED)
            return "RRT-Informed", self.total_nodes, self.total_cost, self.total_collisions, self.total_planning_time, search_vertices, self.total_samples, self.success
        return None, None, None, None, None, None, None, False
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()
