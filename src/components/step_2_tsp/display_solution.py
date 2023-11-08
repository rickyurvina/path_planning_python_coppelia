import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from numpy.distutils.fcompiler import none
from scipy.spatial.distance import cdist
from src.components.common import load_files, save_files
from src.components.common.ordered_positions import get_ordered_positions
from src.components.common.ordered_weigths import get_ordered_weights
from src.components.step_2_tsp import get_row_of_position
from src.steps import config
import matplotlib.patches as mpatches


def plots_positions(route=none, data=none, rows=none, name_folder=none):
    positions = data['positions']
    weights = data['demands']
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
    ax.set_xlim([leftmost_x - 3, rightmost_x + 3])
    ax.set_ylim([lowest_y - 3, highest_y + 3])
    i = 0
    for position, weight in zip(positions, weights):

        color = cmap(norm(weight))
        circle = plt.Circle((position[0], position[1]), 0.9, color=color)
        ax.add_artist(circle)
        colorText = 'white'
        if weight > 3 and weight < 7:
            colorText = 'black'
        weightText = str(weight) + '(Kg)'

        if i < len(positions) - 1:
            start_pos = positions[i]
            end_pos = positions[i + 1]
            actual_row = get_row_of_position.find_row_for_position(rows, start_pos)
            next_row = get_row_of_position.find_row_for_position(rows, end_pos)
            if actual_row == 'h0' or next_row == 'h0':
                weightText = 'Depot'

        ax.text(position[0], position[1], weightText, color=colorText, ha='center', va='center', label='Node')
        i = i + 1
    plt.colorbar(scalar_map, label='Weight scale (Kg)', orientation='vertical', pad=0.01, shrink=0.8)
    total_distance = 0
    for i in range(len(ordered_positions) - 1):
        start_pos = ordered_positions[i]
        end_pos = ordered_positions[i + 1]
        angle_rot = np.arctan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        radius = 0.9
        radius_start = 0.8
        start_x = start_pos[0] + radius_start * np.cos(angle_rot)
        start_y = start_pos[1] + radius_start * np.sin(angle_rot)
        end_x = end_pos[0] - radius * np.cos(angle_rot)
        end_y = end_pos[1] - radius * np.sin(angle_rot)
        arrow = ax.arrow(start_x, start_y, end_x - start_x, end_y - start_y,
                         color='black', linestyle='-', length_includes_head=True, head_width=0.3, label='Route',
                         head_starts_at_zero=True)

        actual_row = get_row_of_position.find_row_for_position(rows, start_pos)
        next_row = get_row_of_position.find_row_for_position(rows, end_pos)
        factor = 1
        if (actual_row != next_row):
            factor = int(config.FACTOR_WEIGHT)
        if actual_row == 'h0' or next_row == 'h0':
            factor = 1
        weight_to_node = 1 if ordered_weights[i + 1] == 0 else ordered_weights[i + 1]
        dist = cdist([start_pos], [end_pos])[0][0]
        custom_cost = dist * factor / weight_to_node
        total_distance += custom_cost

        angle = np.arctan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0]) * 180 / np.pi

        x_pos = (start_x + end_x) / 2
        y_pos = (start_y + end_y) / 2

        if -15 <= angle <= 15 or 165 <= angle <= 195:  # Flecha horizontal (0 grados o 180 grados)
            ax.annotate(f'{custom_cost:.2f}(c)', (x_pos, y_pos + 0.1), ha='center', va='bottom', fontsize=10,
                        color='black',
                        label='Cost')
        else:
            if angle > 90 or angle < -90:
                angle = angle - 180
                ax.annotate(f'{custom_cost:.2f}(c)', (x_pos, y_pos), ha='center', va='bottom', fontsize=10,
                            color='black',
                            label='Cost', rotation=angle)
            else:
                ax.annotate(f'{custom_cost:.2f}(c)', (x_pos, y_pos), ha='left', va='center', fontsize=10, color='black',
                            label='Cost', rotation=angle)

    # Crear la leyenda con la distancia total
    legend_text = f"Total cost: {total_distance:.2f}(c)"
    capacity = f"DDR capacity: {config.VEHICLE_CAPACITIES}(kg)"
    distance_text = mpatches.Patch(color='white', label=legend_text)
    capacity_text = mpatches.Patch(color='white', label=capacity)
    plt.legend(handles=[distance_text, capacity_text, arrow])
    ax.grid(True)
    filename = save_files.get_name_to_save_plot(name_folder, 'tsp_solution_weighted')
    plt.title("Prioritized route for harvesting")
    plt.xlabel('x-coord (m)')
    plt.ylabel('y-coord (m)')
    # plt.savefig(filename, dpi=500)
    print("Saved as", filename)
    plt.show()


if __name__ == '__main__':
    data = load_files.load_solution_data()
    plt = plots_positions(data['route'], data, data['rows'], data['name_folder'])
    print('This script is not meant to be run directly. Run main.py instead.')
