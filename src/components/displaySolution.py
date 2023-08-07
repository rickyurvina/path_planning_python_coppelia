import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from numpy.distutils.fcompiler import none
from scipy.spatial.distance import cdist
from src.components import saveFiles, orderedPositions
from src.components.orderedPositions import get_ordered_positions
from src.components.orderedWeigths import get_ordered_weights
from src.components import getRowOfPosition
from dotenv import load_dotenv
import os

load_dotenv('/src/steps/.env')  # Ruta absoluta


def plots_positions(route=none, data=none, rows=none, name_folder=none):
    positions = data['positions']
    weights = data['demands']
    distance_matrix = data['distance_matrix']
    jet = cm.get_cmap('jet')
    scalar_map = cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=max(weights)), cmap=jet)
    fig, ax = plt.subplots()

    max_weight = np.max(weights)
    cmap = plt.get_cmap('jet')  # obtener el colormap Jet
    norm = plt.Normalize(vmin=0, vmax=max_weight)  # normalizar los pesos
    ordered_positions = get_ordered_positions(route, positions)
    ordered_weights = get_ordered_weights(route, weights)
    leftmost_x = min(positions, key=lambda pos: pos[0])[0]
    rightmost_x = max(positions, key=lambda pos: pos[0])[0]
    lowest_y = min(positions, key=lambda pos: pos[1])[1]
    highest_y = max(positions, key=lambda pos: pos[1])[1]
    ax.set_xlim([leftmost_x - 1, rightmost_x + 1])
    ax.set_ylim([lowest_y - 1, highest_y + 1])
    for position, weight in zip(positions, weights):
        color = cmap(norm(weight))
        circle = plt.Circle((position[0], position[1]), 0.3, color=color)
        ax.add_artist(circle)
        ax.text(position[0], position[1], str(weight), color='black', ha='center', va='center')
    plt.colorbar(scalar_map, label='Weight scale (Kg)')
    total_distance = 0
    for i in range(len(ordered_positions) - 1):
        ax.arrow(ordered_positions[i][0], ordered_positions[i][1],
                 ordered_positions[i + 1][0] - ordered_positions[i][0],
                 ordered_positions[i + 1][1] - ordered_positions[i][1],
                 color='cyan', linestyle='dashed', length_includes_head=True, head_width=0.1)
        start_pos = ordered_positions[i]
        end_pos = ordered_positions[i + 1]
        actual_row = getRowOfPosition.find_row_for_position(rows, start_pos)
        next_row = getRowOfPosition.find_row_for_position(rows, end_pos)
        factor = 1
        if (actual_row != next_row):
            factor = int(int(os.getenv('FACTOR_WEIGHT')))
        if actual_row == 'h0' or next_row == 'h0':
            factor = 1
        weight_to_node = 1 if ordered_weights[i + 1] == 0 else ordered_weights[i + 1]
        dist = cdist([start_pos], [end_pos])[0][0]
        custom_cost = dist * factor / weight_to_node
        total_distance += custom_cost
        x_pos = (start_pos[0] + end_pos[0]) / 2
        y_pos = (start_pos[1] + end_pos[1]) / 2
        ax.annotate(f'{custom_cost:.2f}(c)', (x_pos, y_pos), ha='center', va='center')

    # Crear la leyenda con la distancia total
    legend_text = f"Total cost: {total_distance:.2f}(c)"
    capacity = f"Maximum capacity: {data['vehicle_capacities'][0]}(kg)"
    ax.text(0.005, 0.95, legend_text, transform=ax.transAxes, fontsize=10)
    ax.text(0.5, 0.95, capacity, transform=ax.transAxes, fontsize=10)
    ax.grid(True)
    filename = saveFiles.get_name_to_save_plot(name_folder, 'tsp_solution_weighted')
    plt.title("Visit prioritization (TSP)")
    plt.xlabel('x-coorde (m)')
    plt.ylabel('y-coord (m)')
    plt.savefig(filename, dpi=500)
    print("Saved as", filename)
    plt.show()
