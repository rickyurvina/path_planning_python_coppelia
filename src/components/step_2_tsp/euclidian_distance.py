import math
from src.components.common import load_files
from src.components.step_2_tsp import get_row_of_position
from src.steps import config


def compute_euclidean_distance_matrix(positions, weights, rows):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(positions):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(positions):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                actual_row = get_row_of_position.find_row_for_position(rows, from_node)
                next_row = get_row_of_position.find_row_for_position(rows, to_node)
                factor = 1
                if actual_row != next_row:
                    factor = 2
                distance = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1])) * 100))
                factor_w = 1
                if weights[to_counter] > 0:
                    factor_w = weights[to_counter]
                distances[from_counter][to_counter] = int(distance * factor / factor_w)
    return distances


def compute_custom_cost_matrix(positions, weights, rows):
    """Creates callback to return custom cost between points."""
    custom_costs = {}
    for from_counter, from_node in enumerate(positions):
        custom_costs[from_counter] = {}
        for to_counter, to_node in enumerate(positions):
            if from_counter == to_counter:
                custom_costs[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distance = math.hypot((from_node[0] - to_node[0]), (from_node[1] - to_node[1]))

                # Check if nodes are in different rows and apply the factor
                actual_row = get_row_of_position.find_row_for_position(rows, from_node)
                next_row = get_row_of_position.find_row_for_position(rows, to_node)
                factor = 1
                if actual_row != next_row:
                    factor = int(config.FACTOR_WEIGHT)
                if actual_row == 'h0' or next_row == 'h0':
                    factor = 1

                # Calculate custom cost by combining distance and weight
                weight_to_node = 1 if weights[to_counter] == 0 else weights[to_counter]
                custom_cost = distance * factor / weight_to_node

                custom_costs[from_counter][to_counter] = int(custom_cost * 100)

    return custom_costs


if __name__ == '__main__':
    data = load_files.load_solution_data()
    distances = compute_euclidean_distance_matrix(data['positions'], data['weights'], data['rows'])
    print(distances)
    custom_costs = compute_custom_cost_matrix(data['positions'], data['weights'], data['rows'])
    print(custom_costs)
