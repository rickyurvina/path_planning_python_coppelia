# Import necessary modules
import step_2_tsp
import step_1_get_data
from src.components.common import load_files, save_files
from src.components import create_folder
from src.components.common.save_on_database import save_data_base
import time
import step_3_rrt
from colorama import init, Fore
import config

# Initialize colorama for colored terminal output
init()


def main():
    """
    Main function to execute the steps of the application.
    """
    try:
        # Initialize empty data list
        data = []

        # Start timer for data retrieval
        start_time_data = time.time()

        # Create a new folder
        name_folder = create_folder.create_folder()

        # If the application is set to online mode, get data
        if config.ON_LINE:
            data = step_1_get_data.get_data(name_folder)

        # Calculate execution time for data retrieval
        end_time_data = time.time()
        execution_time_data = end_time_data - start_time_data
        print("execution_time_data", execution_time_data)

        # Start timer for TSP
        start_time_tsp = time.time()

        # Initialize TSP variables
        total_loaded_tsp = 0
        total_length_tsp = 0
        route = ""

        # If the application is set to online mode, execute TSP
        if config.ON_LINE:
            data['ordered_positions'], route, total_loaded_tsp, total_length_tsp = step_2_tsp.main(
                data['positions'],
                data['weights'],
                data['rows'], name_folder)

        # Calculate execution time for TSP
        end_time_tsp = time.time()
        execution_time_tsp = end_time_tsp - start_time_tsp
        print("execution_time_tsp", execution_time_tsp)

        # If the application is set to offline mode, load solution data
        if config.ON_LINE == 0:
            data = load_files.load_solution_data("solutions")
            name_folder = data['name_folder']

        # Initialize paths list
        paths = []

        # Start timer for RRT
        start_time_rrt = time.time()

        # Execute RRT
        self_rrt = step_3_rrt.informed_rrt(
            data,
            name_folder)

        # Append RRT search vertices to paths
        paths.append(self_rrt.search_vertices)

        # Calculate execution time for RRT
        end_time_rrt = time.time()
        execution_time_rrt = end_time_rrt - start_time_rrt
        print("execution_time_rrt", execution_time_rrt)

        # Prepare variables for saving
        variables = {
            'prefix': 'app',
            'rgb': data['rgb'],
            'occupancy_grid': data['occupancy_grid'],
            'positions': data['positions'],
            'weights': data['weights'],
            'demands': data['weights'],
            'rows': data['rows'],
            'ordered_positions': data['ordered_positions'],
            'route': route,
            'paths': paths,
            'path_length_rrt': self_rrt.total_cost,
            'time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'total_loaded_tsp': total_loaded_tsp,
            'total_length_tsp': total_length_tsp,
            'name_folder': name_folder,
            'on_line': config.ON_LINE,
        }

        # Prepare data for saving to database
        data = {
            'total_time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'load_data_time': execution_time_data,
            'tsp_time': execution_time_tsp,
            'rrt_time': execution_time_rrt,
            'folder': name_folder,
            'path_length_rrt': self_rrt.total_cost if self_rrt.success else 0,
            'total_loaded_tsp': total_loaded_tsp if self_rrt.success else 0,
            'total_length_tsp': total_length_tsp if self_rrt.success else 0,
        }

        # Save data to database
        save_data_base(data)

        # Save workspace variables
        save_files.save_workspace(variables, name_folder)

        # Print success message
        print(Fore.LIGHTGREEN_EX + "!!Path founded!!")

    except Exception as e:
        # Print any exceptions that occur
        print(e)


# Execute main function
if __name__ == '__main__':
    main()
