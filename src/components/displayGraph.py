import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from numpy.distutils.fcompiler import none
from scipy.spatial.distance import cdist
from src.components import getDynamicObjectsCoppelia
import pickle

# Cargar las variables guardadas en el archivo .pickle
# with open('../files/routes2.pickle', 'rb') as f:
#     data = pickle.load(f)
#     route = data['route']

def plotsPositions(route=none):
    distance_matrix, weights, positions = getDynamicObjectsCoppelia.getData()
    jet = cm.get_cmap('jet')
    scalar_map = cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=max(weights)), cmap=jet)
# Graficar los cuboides en una escala de color JET y el camino óptimo en amarillo
    fig, ax = plt.subplots()
    ax.set_xlim([1,6])
    ax.set_ylim([1, 7])
    max_weight = np.max(weights)
    cmap = plt.get_cmap('jet') # obtener el colormap Jet
    norm = plt.Normalize(vmin=0, vmax=max_weight) # normalizar los pesos
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])
    for position, weight in zip(positions, weights):
        color = cmap(norm(weight))
        rect = plt.Rectangle((position[0] - 0.1, position[1] - 0.1), 0.3, 0.3, color=color)
        ax.add_patch(rect)
        ax.annotate(f'Peso: {weight}', (position[0] + 0.2, position[1] + 0.2))
    plt.colorbar(scalar_map, label='Peso')

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
        dist = np.linalg.norm(np.array(ordered_positions[i]) - np.array(ordered_positions[i + 1]))
        total_distance += dist

    # Crear la leyenda con la distancia total
    legend_text = f"Distancia total: {total_distance:.2f}"
    ax.text(0.05, -0.1, legend_text, transform=ax.transAxes, fontsize=10)
    plt.show()

# plotsPositions(route)

