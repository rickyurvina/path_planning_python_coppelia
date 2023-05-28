from src.coppelia import sim
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
from matplotlib import cm

def tsp(weights, positions):
    dist_matrix = cdist(positions, positions, metric='euclidean')
    dist_matrix *= np.max(weights) / np.max(dist_matrix)
    row_ind, col_ind = linear_sum_assignment(dist_matrix)
    # Ordenar las posiciones según el camino óptimo
    ordered_positions = []
    for col in col_ind:
        ordered_positions.append(positions[col])
    return ordered_positions

# Establecer la conexión con CoppeliaSim
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    print('No se pudo conectar con CoppeliaSim')
    exit()
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

num_cuboids = 4  # Definir el número de cuboides que estarán presentes en la escena
cuboid_handles = []
for i in range(num_cuboids):
    _, cuboid_handle = sim.simxGetObjectHandle(clientID, f'Cuboid{i + 1}', sim.simx_opmode_blocking)
    cuboid_handles.append(cuboid_handle)

# Obtener la posición y peso de los cuboides
positions = []
weights = []
for cuboid_handle in cuboid_handles:
    _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
    pos = [position[0], position[1]]
    positions.append(pos)
    _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
    weights.append(mass)

ordered_positions = tsp(weights, positions)

# Crear el objeto ScalarMappable y asignar los valores de peso a colores en la escala JET
jet = cm.get_cmap('jet')
scalar_map = cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=max(weights)), cmap=jet)

# Graficar los cuboides en una escala de color JET y el camino óptimo en amarillo
fig, ax = plt.subplots()
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
max_weight = np.max(weights)

cmap = plt.get_cmap('jet') # obtener el colormap Jet
norm = plt.Normalize(vmin=0, vmax=max_weight) # normalizar los pesos
for position, weight in zip(positions, weights):
    color = cmap(norm(weight))
    rect = plt.Rectangle((position[0] - 0.1, position[1] - 0.1), 0.3, 0.3, color=color)
    ax.add_patch(rect)
    ax.annotate(f'Peso: {weight}', (position[0] + 0.2, position[1] + 0.2))

for i in range(len(ordered_positions) - 1):
    start_pos = ordered_positions[i]
    end_pos = ordered_positions[i + 1]
    dist = cdist([start_pos], [end_pos])[0][0]
    x_pos = (start_pos[0] + end_pos[0]) / 2
    y_pos = (start_pos[1] + end_pos[1]) / 2
    ax.annotate(f'{dist:.2f}', (x_pos, y_pos), ha='center', va='center')
    ax.plot([ordered_positions[i][0], ordered_positions[i + 1][0]],
            [ordered_positions[i][1], ordered_positions[i + 1][1]], color='cyan')

total_distance = 0
for i in range(len(ordered_positions) - 1):
    dist = np.linalg.norm(np.array(ordered_positions[i]) - np.array(ordered_positions[i+1]))
    total_distance += dist

# Crear la leyenda con la distancia total
legend_text = f"Distancia total: {total_distance:.2f}"
ax.text(0.05, -0.1, legend_text, transform=ax.transAxes, fontsize=10)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
plt.colorbar(sm)

plt.show()
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(clientID)