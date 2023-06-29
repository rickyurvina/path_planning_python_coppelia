import tsp
import rrt
import getDataCoppelia
from src.components import saveFiles
from src.components import createFolder


def main():
    try:
        path = {}
        name_folder = createFolder.create_folder()
        data = getDataCoppelia.get_data(name_folder)
        ordered_positions, route = tsp.main(data['positions'], data['weights'], data['rows'], name_folder)
        path = rrt.main_rrt(data['occupancy_grid'], ordered_positions, data['rgb'], name_folder)
        variables = {
            'prefix': 'app',
            'rgb': data['rgb'],
            'occupancy_grid': data['occupancy_grid'],
            'positions': data['positions'],
            'weights': data['weights'],
            'rows': data['rows'],
            'ordered_positions': ordered_positions,
            'route': route,
            'path': path
        }

        saveFiles.save_workspace(variables, name_folder)

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
