import numpy as np
from src.coppelia import sim
import math
from numpy.distutils.fcompiler import none
import random

# Costo computacional O(n^2)-> n= len positions
def get_data(num_objects=none, num_rows=none):
    # Costo computacional O(n^2)-> n= len positions
    def compute_euclidean_distance_matrix(positions):
        """Creates callback to return distance between points."""
        distances = {}
        for from_counter, from_node in enumerate(positions):
            distances[from_counter] = {}
            for to_counter, to_node in enumerate(positions):
                if from_counter == to_counter:
                    distances[from_counter][to_counter] = 0
                else:
                    # Euclidean distance
                    distances[from_counter][to_counter] = (int(
                        math.hypot((from_node[0] - to_node[0]),
                                   (from_node[1] - to_node[1])) * 100))
        return distances

    def generate_forbidden_connections(rows):
        forbidden_connections = []
        num_rows = len(rows)

        for h in range(1, num_rows + 1):
            current_row = rows[f'h{h}']
            prev_row = rows[f'h{h - 1}'] if h > 1 else None
            next_row = rows[f'h{h + 1}'] if h < num_rows else None

            for obj_name, (obj_pos, obj_index) in current_row.items():
                if 'c' in obj_name:
                    if prev_row:
                        for prev_obj_name, (prev_obj_pos, prev_obj_index) in prev_row.items():
                            forbidden_connections.append((obj_index, prev_obj_index))
                    if next_row:
                        for next_obj_name, (next_obj_pos, next_obj_index) in next_row.items():
                            forbidden_connections.append((obj_index, next_obj_index))

        return forbidden_connections

    # Establecer la conexión con CoppeliaSim
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print('No se pudo conectar con CoppeliaSim')
        exit()
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    positions = []
    weights = []
    rows = {}
    range_centers = num_objects - num_rows * 4

    for h in range(num_rows):
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

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)
    positions = np.array(positions)
    forbidden_connections = generate_forbidden_connections(rows)

    distance_matrix = compute_euclidean_distance_matrix(positions)

    return distance_matrix, weights, positions, forbidden_connections


# distance_matrix, weights, positions, forbidden_connections = get_data(10, 2)
#
# print(distance_matrix)
# print(positions)