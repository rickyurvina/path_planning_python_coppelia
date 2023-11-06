from src.components.close_simulation_coppelia import close_simulation
from src.components.start_simulation_coppelia import startSimulation
from src.components import get_positions_objects, create_folder, generate_rgb, generate_cccupancy_grid


def get_data(name_folder):
    try:
        clientId = startSimulation()
        positions, weights, rows, object_handles = get_positions_objects.get_positions(clientId)
        occupancy_grid = generate_cccupancy_grid.main(clientId, object_handles, name_folder)
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
