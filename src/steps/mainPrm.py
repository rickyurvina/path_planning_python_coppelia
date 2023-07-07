from PRM import PRM
from src.components import loadFiles
from src.components.shiftPositions import shift_positions
from dotenv import load_dotenv
import os
import traceback
from colorama import init, Fore

load_dotenv()
init()


def uniform():
    try:
        data = loadFiles.load_data_occupancy_grid()
        map_array = data['occupancy_grid']
        ordered_positions = data['ordered_positions']
        ordered_transformed = shift_positions(ordered_positions)
        PRM_planner = PRM(map_array)
        PRM_planner.sample(n_pts=1000, sampling_method="uniform")
        PRM_planner.search(ordered_transformed, "uniform")
    except Exception as e:
        print(Fore.RED + e)
        traceback.print_exc()


uniform()
# def random(map_array, ordered_positions, name_folder):
#     try:
#         ordered_transformed = shift_positions(ordered_positions)
#         PRM_planner = PRM(map_array,name_folder)
#         PRM_planner.sample(n_pts=1000, sampling_method="random")
#         return PRM_planner.search(ordered_transformed, "random")
#     except Exception as e:
#         print(Fore.RED + e)
#         traceback.print_exc()
#
#
# def select_method(map_array, ordered_positions, name_folder):
#     method = os.getenv('METHOD')
#     if method == 'uniform':
#         return random(map_array, ordered_positions, name_folder)
#     elif method == 'uniform':
#         return uniform(map_array, ordered_positions, name_folder)
#     else:
#         raise Exception('Method not found')
