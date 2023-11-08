from src.components.step_3_rrt.informed_rrt import RRT
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files
from src.components import create_folder
import os
import traceback
from colorama import init, Fore

init()


def informed_rrt(data, name_folder):
    try:
        ordered_transformed = shift_positions(data['ordered_positions'])
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'],
                          data['ordered_positions'], name_folder)
        return RRT_planner.informed_RRT_star()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def informed_rrt_unicycle(data, name_folder):
    try:
        ordered_transformed = shift_positions(data['ordered_positions'])
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'],
                          data['ordered_positions'], name_folder)
        return RRT_planner.informed_RRT_star_unicycle()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def rrt(map_array, ordered_positions, rows, name_folder):
    try:
        ordered_transformed = shift_positions(ordered_positions)
        RRT_planner = RRT(map_array, ordered_transformed, rows, ordered_positions, name_folder)
        return RRT_planner.RRT()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def rrt_star(map_array, ordered_positions, rows, name_folder):
    try:
        ordered_transformed = shift_positions(ordered_positions)
        RRT_planner = RRT(map_array, ordered_transformed, rows, ordered_positions, name_folder)
        return RRT_planner.RRT_star()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def select_method(map_array, ordered_positions, rows, name_folder):
    method = os.getenv('METHOD')
    if method == 'RRT':
        return rrt(map_array, ordered_positions, rows, name_folder)
    elif method == 'INFORMED_RRT':
        return informed_rrt(map_array, ordered_positions, rows, name_folder)
    elif method == 'RRT_STAR':
        return rrt_star(map_array, ordered_positions, rows, name_folder)
    elif method == 'INFORMED_RRT_UNICYCLE':
        return informed_rrt_unicycle(map_array, ordered_positions, rows, name_folder)
    else:
        raise Exception('Method not found')


if __name__ == '__main__':
    data = load_files.load_solution_data("solutions")
    name_folder = create_folder.create_folder()
    print(data)
    informed_rrt(data, name_folder)
