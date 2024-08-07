# Import necessary modules and configurations
from src.components.step_3_rrt.informed_rrt import RRT
from src.components.step_3_rrt.shift_positions import shift_positions
from src.components.common import load_files
from src.components import create_folder
import os
import traceback
from colorama import init, Fore
from src.steps import config

# Initialize colorama for colored terminal output
init()


def rrt(data, name_folder, path_solutions=config.PATH_FOLDER):
    """
    Function to execute the RRT algorithm.
    Args:
        data (dict): The data dictionary.
        name_folder (str): The name of the folder where the data will be stored.
        path_solutions (str): The path to the solutions folder.
    Returns:
        RRT: An instance of the RRT class.
    """
    try:
        # Transform the ordered positions
        ordered_transformed = shift_positions(data['ordered_positions'])

        # Initialize the RRT planner
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'],
                          data['ordered_positions'], name_folder, path_solutions)

        # Execute the RRT algorithm
        return RRT_planner.rrt()
    except Exception as e:
        # Print any exceptions that occur
        print(Fore.RED + str(e))
        traceback.print_exc()


def rrt_star(data, name_folder, path_solutions=config.PATH_FOLDER):
    """
      Function to execute the RRT* algorithm.
      Args:
          data (dict): The data dictionary.
          name_folder (str): The name of the folder where the data will be stored.
          path_solutions (str): The path to the solutions folder.
      Returns:
          RRT: An instance of the RRT class.
      """
    try:
        ordered_transformed = shift_positions(data['ordered_positions'])
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'],
                          data['ordered_positions'], name_folder, path_solutions)
        return RRT_planner.rrt_star()
    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()


def informed_rrt(data, name_folder, path_solutions=config.PATH_FOLDER):
    """
      Function to execute the IRRT* algorithm.
      Args:
          data (dict): The data dictionary.
          name_folder (str): The name of the folder where the data will be stored.
          path_solutions (str): The path to the solutions folder.
      Returns:
          RRT: An instance of the RRT class.
      """
    try:
        ordered_transformed = shift_positions(data['ordered_positions'])
        RRT_planner = RRT(data['occupancy_grid'], data['rgb'], ordered_transformed, data['rows'],
                          data['ordered_positions'], name_folder, path_solutions)
        return RRT_planner.rrt_informed()
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


def select_method(map_array, ordered_positions, rows, name_folder):
    """
    Function to select the method to use based on the environment variable 'METHOD'.
    Args:
        map_array (list): The map array.
        ordered_positions (list): The ordered positions.
        rows (int): The number of rows.
        name_folder (str): The name of the folder where the data will be stored.
    Returns:
        RRT: An instance of the RRT class.
    """
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
    # Load the solution data
    data = load_files.load_solution_data("solutions")

    # Create a new folder
    name_folder = create_folder.create_folder()

    # Get the path to the solutions folder
    path_solutions = config.PATH_FOLDER

    # Execute the informed RRT algorithm
    # rrt(data, name_folder, path_solutions)
    # rrt_star(data, name_folder, path_solutions)
    informed_rrt(data, name_folder, path_solutions)
