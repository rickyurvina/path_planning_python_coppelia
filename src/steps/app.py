import tsp
import rrt
import getDataCoppelia
from src.components import saveFiles
from src.components import createFolder
from saveOnDatabase import save_data_base
import time
from src.components import loadFiles
import mainInformedRrt
from dotenv import load_dotenv
import os
load_dotenv()


def main():
    try:
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
        if os.getenv('ON_LINE') == '1':
            ordered_positions, route = tsp.main(data['positions'], data['weights'], data['rows'], name_folder)
        end_time_tsp = time.time()
        execution_time_tsp = end_time_tsp - start_time_tsp
        print("execution_time_tsp", execution_time_tsp)

        if os.getenv('ON_LINE') == '0':
            data = loadFiles.load_data_occupancy_grid()
        # RRT
        start_time_rrt = time.time()
        method = os.getenv('METHOD')
        if method == 'RRT':
            path = rrt.main_rrt(data['occupancy_grid'], data['ordered_positions'], data['rgb'], name_folder)
        elif method == 'INFORMED_RRT':
            path = mainInformedRrt.main_informed_rrt(data['occupancy_grid'], data['ordered_positions'], name_folder)
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
            'route': path,
            'path': path
        }

        data = {
            'total_time': execution_time_data + execution_time_tsp + execution_time_rrt,
            'load_data_time': execution_time_data,
            'tsp_time': execution_time_tsp,
            'rrt_time': execution_time_rrt,
            'folder': name_folder,
        }
        save_data_base(data)
        saveFiles.save_workspace(variables, name_folder)

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
