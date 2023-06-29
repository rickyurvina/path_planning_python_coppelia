import numpy as np
from src.coppelia import sim
import random
from dotenv import load_dotenv
import os
load_dotenv()
# Costo computacional O(n^2)-> n= len positions
def get_positions(clientID):

    positions = []
    weights = []
    rows = {}

    for h in range(int(os.getenv('NUM_ROWS'))):
        row_name = f'h{h + 1}'
        rows[row_name] = {}
        for o in range(4):
            object_name = f'hi{h + 1}b{o + 1}'
            _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            # _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
            # if (mass > 0):
            # weights.append(int(mass))
            weights.append(random.randint(3, 9))
            _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
            pos = [position[0], position[1]]
            positions.append(pos)
            rows[row_name][object_name] = pos, len(positions) - 1
        for o in range(1):
            object_name = f'hi{h + 1}c{o + 1}'
            _, cuboid_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            # _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
            # if (mass > 0):
            weights.append(random.randint(3, 9))
            # weights.append(int(mass))
            _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
            pos = [position[0], position[1]]
            positions.append(pos)
            rows[row_name][object_name] = pos, len(positions) - 1

    positions = np.array(positions)

    return positions, weights, rows

