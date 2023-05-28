from src.coppelia import sim
import numpy as np
import math


def getData():
    def distance_matrix(positions):
        n = len(positions)
        distances = [[0] * n for i in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    dx = positions[i][0] - positions[j][0]
                    dy = positions[i][1] - positions[j][1]
                    distances[i][j] = math.sqrt(dx * dx + dy * dy)
        return distances

    # Establecer la conexión con CoppeliaSim
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print('No se pudo conectar con CoppeliaSim')
        exit()
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    num_cuboids = 4  # Definir el número de cuboides que estarán presentes en la escena
    cuboid_handles = []
    positions = []
    weights = []
    for i in range(num_cuboids):
        _, cuboid_handle = sim.simxGetObjectHandle(clientID, f'Cuboid{i + 1}', sim.simx_opmode_blocking)
        cuboid_handles.append(cuboid_handle)
        _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
        pos = [round(position[0], 1), round(position[1], 1)]
        positions.append(pos)
        _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
        weights.append(round(float(mass), 1))

    positions = np.array(positions)
    distance_matrix = distance_matrix(positions)

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)

    return distance_matrix, weights, positions


distance_matrix, weights, positions = getData()
