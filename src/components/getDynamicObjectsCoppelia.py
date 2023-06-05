from src.coppelia import sim
import numpy as np
import math
from numpy.distutils.fcompiler import none


def getData(num_objects=none):
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
                                   (from_node[1] - to_node[1]))))
        return distances

    # Establecer la conexión con CoppeliaSim
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print('No se pudo conectar con CoppeliaSim')
        exit()
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    num_cuboids = num_objects if num_objects else 40  # Definir el número de cuboides que estarán presentes en la escena
    cuboid_handles = []
    positions = []
    weights = []
    for i in range(num_cuboids):
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, f'Obj{i}', sim.simx_opmode_blocking)
        cuboid_handles.append(cuboid_handle)
        _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
        # if mass > 0:
        weights.append(int(mass))
        _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
        pos = [round(position[0], 2), round(position[1], 2)]
        positions.append(pos)

    positions = np.array(positions)
    distance_matrix = compute_euclidean_distance_matrix(positions)

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)

    return distance_matrix, weights, positions

# distance_matrix, weights, positions = getData()
