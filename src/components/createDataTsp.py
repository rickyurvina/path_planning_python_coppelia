from src.components import euclidianDistance, loadFiles
from src.components import forbiddenConnections
from src.steps import config


# Costo computacional O(1)
def create_data_model(positions, weights, rows):
    """Stores the data for the problem."""
    distance_matrix = euclidianDistance.compute_custom_cost_matrix(positions, weights, rows)
    forbidden_connections = forbiddenConnections.generate_forbidden_connections(rows)
    data = {
        'positions': positions,
        'distance_matrix': distance_matrix,
        'demands': weights,
        'vehicle_capacities': [int(config.VEHICLE_CAPACITIES)],
        'num_vehicles': 1,
        'depot': 0,
        'forbidden_connections': forbidden_connections,
    }
    return data


def main():
    data = loadFiles.load_solution_data()
    data = create_data_model(data['positions'], data['weights'], data['rows'])
    print(data)
    return data


if __name__ == '__main__':
    main()
