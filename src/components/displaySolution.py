import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from numpy.distutils.fcompiler import none
from scipy.spatial.distance import cdist
from src.components import saveFiles, orderedPositions
from src.components.orderedPositions import get_ordered_positions

def plots_positions(route=none, data=none):
    positions = data['positions']
    weights = data['demands']
    jet = cm.get_cmap('jet')
    scalar_map = cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=max(weights)), cmap=jet)
    fig, ax = plt.subplots()
    ax.set_xlim([-5, 1])
    ax.set_ylim([-6, 0])

    max_weight = np.max(weights)
    cmap = plt.get_cmap('jet')  # obtener el colormap Jet
    norm = plt.Normalize(vmin=0, vmax=max_weight)  # normalizar los pesos
    ordered_positions = get_ordered_positions(route, positions)

    for position, weight in zip(positions, weights):
        color = cmap(norm(weight))
        circle = plt.Circle((position[0], position[1]), 0.3, color=color)
        ax.add_artist(circle)
        ax.text(position[0], position[1], str(weight), color='black', ha='center', va='center')
    plt.colorbar(scalar_map, label='Escala de Peso')
    total_distance = 0
    for i in range(len(ordered_positions) - 1):
        ax.arrow(ordered_positions[i][0], ordered_positions[i][1],
                 ordered_positions[i + 1][0] - ordered_positions[i][0],
                 ordered_positions[i + 1][1] - ordered_positions[i][1],
                 color='cyan', linestyle='dashed', length_includes_head=True, head_width=0.1)
        dist = np.linalg.norm(np.array(ordered_positions[i]) - np.array(ordered_positions[i + 1]))
        total_distance += dist
        start_pos = ordered_positions[i]
        end_pos = ordered_positions[i + 1]
        dist = cdist([start_pos], [end_pos])[0][0]
        x_pos = (start_pos[0] + end_pos[0]) / 2
        y_pos = (start_pos[1] + end_pos[1]) / 2
        ax.annotate(f'{dist:.2f}', (x_pos, y_pos), ha='center', va='center')

    # Crear la leyenda con la distancia total
    legend_text = f"Distancia total: {total_distance:.2f}"
    capacity = f"Capacidad: {data['vehicle_capacities']}"
    restriction = f"Restriccón: {data['forbidden_connections']}"
    ax.text(0.05, -0.1, legend_text, transform=ax.transAxes, fontsize=10)
    ax.text(0.5, -0.1, capacity, transform=ax.transAxes, fontsize=10)
    ax.text(0.9, -0.1, restriction, transform=ax.transAxes, fontsize=10)
    ax.grid(True)
    filename = saveFiles.get_name_to_save_plot()
    plt.savefig(filename, dpi=500)
    print("Guardado como", filename)
    plt.title("TSP Ordered Harvesting postions")
    plt.show()
