import numpy as np

from src.components.step1_get_data.close_simulation_coppelia import close_simulation
from src.components.step1_get_data.start_simulation_coppelia import startSimulation
from src.coppelia import sim
from src.steps import config


# Costo computacional O(n^2)-> n= len positions
def get_positions(clientID):
    positions = []
    weights = []
    object_handles = []
    rows = {}

    for h in range(int(config.NUM_ROWS)):
        if h == 0 and config.HUSKY == 1:
            row_name = f'h{h}'
            rows[row_name] = {}
            object_name = 'Husky'
            _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            object_handles.append(cuboid_handle)
            weights.append(0)
            _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
            pos = [position[0], position[1]]
            positions.append(pos)
            rows[row_name][object_name] = pos, len(positions) - 1

        row_name = f'h{h + 1}'
        rows[row_name] = {}
        for o in range(int(config.NUM_POINTS_BORDER_BY_ROW)):
            object_name = f'hi{h + 1}b{o + 1}'
            _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
            weights.append(int(mass))
            object_handles.append(cuboid_handle)
            _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
            pos = [position[0], position[1]]
            positions.append(pos)
            rows[row_name][object_name] = pos, len(positions) - 1
        for o in range(int(config.NUM_POINTS_CENTER_BY_ROW)):
            object_name = f'hi{h + 1}c{o + 1}'
            _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)

            weights.append(int(mass))
            object_handles.append(cuboid_handle)
            _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
            pos = [position[0], position[1]]
            positions.append(pos)
            rows[row_name][object_name] = pos, len(positions) - 1

    positions = np.array(positions)

    return positions, weights, rows, object_handles


def get_positions_of_objects_to_hide(clientID):
    object_handles = []
    terrain_handles = []
    husky_name = f'Husky'
    _, cuboid_handle = sim.simxGetObjectHandle(clientID, husky_name, sim.simx_opmode_blocking)
    object_handles.append(cuboid_handle)
    for o in range(int(config.NUM_WAY_POINTS)):
        object_name = f'hi{o + 1}b1'
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
        object_handles.append(cuboid_handle)
    for o in range(int(config.NUM_ROWS_MAP)):
        row_name = f'h{o + 1}'
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, row_name, sim.simx_opmode_blocking)
        object_handles.append(cuboid_handle)
    for o in range(int(config.NUM_BOXES)):
        box_name = f'box{o + 1}'
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, box_name, sim.simx_opmode_blocking)
        object_handles.append(cuboid_handle)

    for o in range(int(config.NUM_TERRAINS)):
        hash_name = f'#{o - 1}' if o > 0 else ''
        tra_name = f'tra{o + 1}{hash_name}'
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, tra_name, sim.simx_opmode_blocking)

        terrain_handles.append(cuboid_handle)

    return object_handles, terrain_handles


if __name__ == '__main__':
    clientId = startSimulation()
    object_handles, terrain_handles = get_positions_of_objects_to_hide(clientId)
    # print(object_handles)
    print(terrain_handles)
    close_simulation(clientId)
