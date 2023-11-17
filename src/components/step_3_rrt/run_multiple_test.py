import uuid
from datetime import datetime
import traceback
from colorama import init, Fore
from src.components.common.save_on_database import save_data_rrt_test
from src.components.step_3_rrt.informed_rrt import RRT
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files, save_files
from src.components import create_folder
from src.reports.reports import average_test
from src.steps import config


def run_multiple_test(num_tests=1):
    try:
        unique_code = str(uuid.uuid4())
        test_number = datetime.now().strftime("%Y%m%d") + "-" + str(unique_code)
        data_loaded = load_files.load_solution_data()
        ordered_transformed = shift_positions(data_loaded['ordered_positions'])
        name_folder = create_folder.create_folder("../../solutions")
        path_solutions = "../../solutions"
        for i in range(num_tests):
            print("Test number: ", i)
            print("Start informed RRT Unicycle star planning")
            RRT_PLANNER = RRT(data_loaded['occupancy_grid'], data_loaded['rgb'], ordered_transformed,
                              data_loaded['rows'],
                              data_loaded['ordered_positions'],
                              name_folder, path_solutions)

            method, total_nodes, total_cost, total_collisions, total_planning_time, _, total_samples, success = RRT_PLANNER.rrt()
            data = {
                'prefix': 'tests_rrt',
                'method': method if method is not None else "tests_rrt",
                'test_number': test_number,
                'total_cost': total_cost if total_cost is not None else 0,
                'total_collisions': total_collisions if total_collisions is not None else 0,
                'total_samples': total_samples if total_samples is not None else 0,
                'total_planning_time': total_planning_time if total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": success,
                "total_nodes": total_nodes if total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt!!")

            method, total_nodes, total_cost, total_collisions, total_planning_time, _, total_samples, success = RRT_PLANNER.rrt_star()
            data = {
                'prefix': 'tests_rrt_star',
                'method': method if method is not None else "rrt_star",
                'test_number': test_number,
                'total_cost': total_cost if total_cost is not None else 0,
                'total_collisions': total_collisions if total_collisions is not None else 0,
                'total_samples': total_samples if total_samples is not None else 0,
                'total_planning_time': total_planning_time if total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": success,
                "total_nodes": total_nodes if total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
            print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt-star!!")

            method, total_nodes, total_cost, total_collisions, total_planning_time, _, total_samples, success = RRT_PLANNER.rrt_informed()
            data = {
                'prefix': 'tests_rrt_star_informed',
                'method': method if method is not None else "tests_rrt_star_informed",
                'test_number': test_number,
                'total_cost': total_cost if total_cost is not None else 0,
                'total_collisions': total_collisions if total_collisions is not None else 0,
                'total_samples': total_samples if total_samples is not None else 0,
                'total_planning_time': total_planning_time if total_planning_time is not None else 0,
                "waypoints_number": config.BREAK_AT if config.BREAK_AT < len(
                    data_loaded['ordered_positions']) else len(
                    data_loaded['ordered_positions']),
                "name_folder": name_folder,
                "success": success,
                "total_nodes": total_nodes if total_nodes is not None else 0,
            }
            save_data_rrt_test(data)
            save_files.save_workspace(data, name_folder, path_solutions)
        print(Fore.LIGHTGREEN_EX + "!!Saves on database rrt-informed!!")
        average_test(test_number, name_folder, path_solutions, num_tests)
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


if __name__ == '__main__':
    run_multiple_test()
