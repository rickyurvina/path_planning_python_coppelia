from src.components.step1_get_data.close_simulation_coppelia import close_simulation
from src.components.step1_get_data.start_simulation_coppelia import startSimulation
from src.components.step1_get_data import generate_cccupancy_grid, generate_rgb, get_positions_objects
from src.components import create_folder


def get_data(name_folder):
    try:
        clientId = startSimulation()
        positions, weights, rows, object_handles = get_positions_objects.get_positions(clientId)
        occupancy_grid = generate_cccupancy_grid.main(clientId, object_handles)
        rgb = generate_rgb.generate_rgb(clientId, name_folder)
        close_simulation(clientId)

        return {
            'weights': weights,
            'positions': positions,
            'rows': rows,
            'occupancy_grid': occupancy_grid,
            'rgb': rgb
        }
    except Exception as e:
        print(e)


def main():
    name_folder = create_folder.create_folder()
    data = get_data(name_folder)
    print(data)
    return data


if __name__ == '__main__':
    main()
