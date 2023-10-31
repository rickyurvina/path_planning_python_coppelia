from src.components.closeSimulationCoppelia import closeSimulation
from src.components.startSimulationCoppelia import startSimulation
from src.coppelia import sim
import generateOccupancyGrid
from src.components import getPositionsObjects, createFolder
import generateRGB


def get_data(name_folder):
    try:
        clientId = startSimulation()
        positions, weights, rows, object_handles = getPositionsObjects.get_positions(clientId)
        occupancy_grid = generateOccupancyGrid.generate_occupancy(clientId, object_handles, name_folder)
        rgb = generateRGB.generate_rgb(clientId, name_folder)
        closeSimulation(clientId)

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
    name_folder = createFolder.create_folder()
    data = get_data(name_folder)
    print(data)
    return data

if __name__ == '__main__':
    main()