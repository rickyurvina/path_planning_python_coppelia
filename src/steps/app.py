import step_2_tsp
import step_1_get_data
from src.components.common import load_files, save_files
from src.components import create_folder
from src.components.common.save_on_database import save_data_base
import time
import step_3_rrt
from colorama import init, Fore
import config

init()


def main():
    try:
        data = []
        # GET DATA
        start_time_data = time.time()
        name_folder = create_folder.create_folder()
        if config.ON_LINE:
            data = step_1_get_data.get_data(name_folder)
        end_time_data = time.time()
        execution_time_data = end_time_data - start_time_data
        print("execution_time_data", execution_time_data)

        # TSP
        start_time_tsp = time.time()
        total_loaded_tsp = 0
        total_length_tsp = 0
        route = ""
        if config.ON_LINE:
            data['ordered_positions'], route, total_loaded_tsp, total_length_tsp = step_2_tsp.main(
                data['positions'],
                data['weights'],
                data['rows'], name_folder)
        end_time_tsp = time.time()
        execution_time_tsp = end_time_tsp - start_time_tsp
        print("execution_time_tsp", execution_time_tsp)

        if config.ON_LINE == 0:
            data = load_files.load_solution_data("solutions")
        paths = []
        # RRT
        start_time_rrt = time.time()
        method, total_nodes, total_cost, total_samples_in_object, total_planning_time, search_vertices = step_3_rrt.informed_rrt(
            data,
            name_folder)
        paths.append(search_vertices)
        end_time_rrt = time.time()
        execution_time_rrt = end_time_rrt - start_time_rrt
        print("execution_time_rrt", execution_time_rrt)

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
            'path_length_rrt': total_cost,
            'time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'total_loaded_tsp': total_loaded_tsp,
            'total_length_tsp': total_length_tsp,
            'name_folder': name_folder,
            'on_line': config.ON_LINE,
        }

        data = {
            'total_time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'load_data_time': execution_time_data,
            'tsp_time': execution_time_tsp,
            'rrt_time': execution_time_rrt,
            'folder': name_folder,
            'path_length_rrt': total_cost,
            'total_loaded_tsp': total_loaded_tsp,
            'total_length_tsp': total_length_tsp,
        }
        save_data_base(data)
        save_files.save_workspace(variables, name_folder)
        print(Fore.LIGHTGREEN_EX + "!!Path founded!!")

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
