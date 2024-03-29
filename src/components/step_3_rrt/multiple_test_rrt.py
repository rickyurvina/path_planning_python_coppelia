import uuid
from datetime import datetime
import traceback

import numpy as np
from colorama import init, Fore
from src.components.common.save_on_database import save_data_rrt_test
from src.components.step_3_rrt.informed_rrt import RRT
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files, save_files
from src.components import create_folder
from src.reports.reports import average_test, success_failure_rate_by_method
from src.steps import config


def run_multiple_test(num_tests=1):
    try:
        unique_code = str(uuid.uuid4())
        test_number = datetime.now().strftime("%Y%m%d") + "-" + str(unique_code)
        data_loaded = load_files.load_solution_data()
        ordered_transformed = shift_positions(data_loaded['ordered_positions'])
        indexes = [0, 1]
        array_for_test = [ordered_transformed[i] for i in indexes]
        name_folder = create_folder.create_folder("../../solutions")
        path_solutions = "../../solutions"
        for i in range(num_tests):
            print("Test number: ", i)
            print("Start informed RRT Unicycle star planning")
            RRT_PLANNER = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                              data_loaded['rows'],
                              data_loaded['ordered_positions'],
                              name_folder, path_solutions)

            self_rrt = RRT_PLANNER.rrt()
            data = {
                'prefix': 'tests_rrt',
                'method': self_rrt.name_method if self_rrt is not None else "RRT",
                'test_number': test_number,
                'total_cost': self_rrt.total_cost if self_rrt is not None else 0,
                'total_collisions': self_rrt.total_collisions if self_rrt is not None else 0,
                'total_samples': self_rrt.total_samples if self_rrt is not None else 0,
                'total_planning_time': self_rrt.total_planning_time if self_rrt is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": self_rrt.success if self_rrt is not None else 0,
                "total_nodes": self_rrt.total_nodes if self_rrt is not None else 0,
                'average_distance': np.mean(self_rrt.distances_obstacles) if self_rrt else 0,
                'std_dev_distance': np.std(self_rrt.distances_obstacles) if self_rrt else 0,
                'max_distance': np.max(self_rrt.distances_obstacles) if self_rrt else 0,
                'min_distance': np.min(self_rrt.distances_obstacles) if self_rrt else 0,
                'variance_distance': np.var(self_rrt.distances_obstacles) if self_rrt else 0,
                'smoothness': self_rrt.smoothness if self_rrt else 0,
                'curvature': self_rrt.curvature if self_rrt else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt!!")

            RRT_STAR_PLANNER = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                                   data_loaded['rows'],
                                   data_loaded['ordered_positions'],
                                   name_folder, path_solutions)

            self_rrt_star = RRT_STAR_PLANNER.rrt_star()
            data = {
                'prefix': 'tests_rrt_star',
                'method': self_rrt_star.name_method if self_rrt_star is not None else "RRT-Star",
                'test_number': test_number,
                'total_cost': self_rrt_star.total_cost if self_rrt_star is not None else 0,
                'total_collisions': self_rrt_star.total_collisions if self_rrt_star is not None else 0,
                'total_samples': self_rrt_star.total_samples if self_rrt_star is not None else 0,
                'total_planning_time': self_rrt_star.total_planning_time if self_rrt_star is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": self_rrt_star.success if self_rrt_star is not None else 0,
                "total_nodes": self_rrt_star.total_nodes if self_rrt_star is not None else 0,
                'average_distance': np.mean(self_rrt_star.distances_obstacles) if self_rrt_star else 0,
                'std_dev_distance': np.std(self_rrt_star.distances_obstacles) if self_rrt_star else 0,
                'max_distance': np.max(self_rrt_star.distances_obstacles) if self_rrt_star else 0,
                'min_distance': np.min(self_rrt_star.distances_obstacles) if self_rrt_star else 0,
                'variance_distance': np.var(self_rrt_star.distances_obstacles) if self_rrt_star else 0,
                'smoothness': self_rrt_star.smoothness if self_rrt_star else 0,
                'curvature': self_rrt_star.curvature if self_rrt_star else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt-star!!")
            RRT_PLANNER_INFORMED = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], array_for_test,
                                       data_loaded['rows'],
                                       data_loaded['ordered_positions'],
                                       name_folder, path_solutions)
            self_rrt_informed = RRT_PLANNER_INFORMED.rrt_informed()
            data = {
                'prefix': 'tests_rrt_star_informed',
                'method': getattr(self_rrt_informed, 'name_method', 'IRRT'),
                'test_number': test_number,
                'total_cost': getattr(self_rrt_informed, 'total_cost', 0),
                'total_collisions': getattr(self_rrt_informed, 'total_collisions', 0),
                'total_samples': getattr(self_rrt_informed, 'total_samples', 0),
                'total_planning_time': getattr(self_rrt_informed, 'total_planning_time', 0),
                'waypoints_number': min(config.BREAK_AT, len(data_loaded['ordered_positions'])),
                'name_folder': name_folder,
                'success': getattr(self_rrt_informed, 'success', 0),
                'total_nodes': getattr(self_rrt_informed, 'total_nodes', 0),
                'average_distance': np.mean(self_rrt_informed.distances_obstacles) if self_rrt_informed else 0,
                'std_dev_distance': np.std(self_rrt_informed.distances_obstacles) if self_rrt_informed else 0,
                'max_distance': np.max(self_rrt_informed.distances_obstacles) if self_rrt_informed else 0,
                'min_distance': np.min(self_rrt_informed.distances_obstacles) if self_rrt_informed else 0,
                'variance_distance': np.var(self_rrt_informed.distances_obstacles) if self_rrt_informed else 0,
                'smoothness': self_rrt_informed.smoothness if self_rrt_informed else 0,
                'curvature': self_rrt_informed.curvature if self_rrt_informed else 0,
            }

            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
        print(Fore.LIGHTGREEN_EX + "!!Saves on database IRRT!!")
        average_test(test_number, name_folder, path_solutions, num_tests)
        # success_failure_rate_by_method(test_number, name_folder, path_solutions, num_tests)
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


if __name__ == '__main__':
    run_multiple_test(150)
