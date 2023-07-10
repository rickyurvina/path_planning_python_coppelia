import tsp
import getDataCoppelia
from src.components import saveFiles
from src.components import createFolder
from saveOnDatabase import save_data_base
import time
from src.components import loadFiles
import mainRrt
from dotenv import load_dotenv
import os
from colorama import init, Fore

init()

load_dotenv()


def main():
    try:
        data = []
        # GET DATA
        start_time_data = time.time()
        name_folder = createFolder.create_folder()
        if os.getenv('ON_LINE') == '1':
            data = getDataCoppelia.get_data(name_folder)
        end_time_data = time.time()
        execution_time_data = end_time_data - start_time_data
        print("execution_time_data", execution_time_data)

        # TSP
        start_time_tsp = time.time()
        total_loaded_tsp = 0
        total_length_tsp = 0
        route = ""
        if os.getenv('ON_LINE') == '1':
            data['ordered_positions'], route, total_loaded_tsp, total_length_tsp = tsp.main(data['positions'],
                                                                                            data['weights'],
                                                                                            data['rows'], name_folder)
        end_time_tsp = time.time()
        execution_time_tsp = end_time_tsp - start_time_tsp
        print("execution_time_tsp", execution_time_tsp)

        if os.getenv('ON_LINE') == '0':
            data = loadFiles.load_solution_data()
        paths = []
        # RRT
        start_time_rrt = time.time()
        path_rrt, path_length_rrt = mainRrt.select_method(data['occupancy_grid'], data['ordered_positions'],
                                                          data['rows'],
                                                          name_folder)
        paths.append(path_rrt)
        end_time_rrt = time.time()
        execution_time_rrt = end_time_rrt - start_time_rrt
        print("execution_time_rrt", execution_time_rrt)

        variables = {
            'prefix': 'app',
            'rgb': data['rgb'],
            'occupancy_grid': data['occupancy_grid'],
            'positions': data['positions'],
            'weights': data['weights'],
            'rows': data['rows'],
            'ordered_positions': data['ordered_positions'],
            'route': route,
            'paths': paths,
            'path_length_rrt': path_length_rrt,
            'time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'total_loaded_tsp': total_loaded_tsp,
            'total_length_tsp': total_length_tsp,
            'folder': name_folder,
            'on_line': os.getenv('ON_LINE')
        }

        data = {
            'total_time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'load_data_time': execution_time_data,
            'tsp_time': execution_time_tsp,
            'rrt_time': execution_time_rrt,
            'folder': name_folder,
            'path_length_rrt': path_length_rrt,
            'total_loaded_tsp': total_loaded_tsp,
            'total_length_tsp': total_length_tsp,
        }
        save_data_base(data)
        saveFiles.save_workspace(variables, name_folder)
        print(Fore.LIGHTGREEN_EX + "!!Path founded!!")

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
