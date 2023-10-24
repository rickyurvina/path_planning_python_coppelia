from RRT_C import RRT
from src.components.shiftPositions import shift_positions
from src.components import loadFiles
from dotenv import load_dotenv
import os
import traceback
from colorama import init, Fore

load_dotenv()
init()


def main_informed_rrt(data,  name_folder):
    try:
        ordered_transformed = shift_positions(data['ordered_positions'])
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'],ordered_transformed, data['rows'], data['ordered_positions'], name_folder)
        return RRT_planner.informed_RRT_star()
    except Exception as e:
        print(Fore.RED + e)
        traceback.print_exc()


def main_rrt(map_array, ordered_positions, rows, name_folder):
    try:
        ordered_transformed = shift_positions(ordered_positions)
        RRT_planner = RRT(map_array, ordered_transformed, rows, ordered_positions, name_folder)
        return RRT_planner.RRT()
    except Exception as e:
        print(Fore.RED + e)
        traceback.print_exc()


def main_rrt_star(map_array, ordered_positions, rows, name_folder):
    try:
        ordered_transformed = shift_positions(ordered_positions)
        RRT_planner = RRT(map_array, ordered_transformed, rows, ordered_positions, name_folder)
        return RRT_planner.RRT_star()
    except Exception as e:
        print(Fore.RED + e)
        traceback.print_exc()


def select_method(map_array, ordered_positions, rows, name_folder):
    method = os.getenv('METHOD')
    if method == 'RRT':
        return main_rrt(map_array, ordered_positions, rows, name_folder)
    elif method == 'INFORMED_RRT':
        return main_informed_rrt(map_array, ordered_positions, rows, name_folder)
    elif method == 'RRT_STAR':
        return main_rrt_star(map_array, ordered_positions, rows, name_folder)
    else:
        raise Exception('Method not found')
