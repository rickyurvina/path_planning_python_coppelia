from src.coppelia import sim
import generateOccupancyGrid
from src.components import getPositionsObjects
import generateRGB


def startSimulation():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    try:
        if clientID == -1:
            print('No se pudo conectar con CoppeliaSim')
            exit()
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
        return clientID
    except Exception as e:
        print(e)
        if clientID != -1:
            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
            sim.simxFinish(clientID)


def closeSimulation(clientID):
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)


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
