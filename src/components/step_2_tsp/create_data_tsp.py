from src.components.step_2_tsp import euclidian_distance, generate_forbidden_connections
from src.steps import config
from src.components.common import load_files


def create_data_model(positions, weights, rows):
    """
    Function to store the data for the problem.

    Parameters:
    positions (list): The positions of the nodes.
    weights (list): The weights of the nodes.
    rows (int): The number of rows in the grid.

    Returns:
    dict: A dictionary containing the data for the problem.
    """
    # Compute the cost matrix
    distance_matrix = euclidian_distance.compute_custom_cost_matrix(positions, weights, rows)
    # Generate the forbidden connections
    forbidden_connections = generate_forbidden_connections.generate_forbidden_connections(rows)
    # Store the data in a dictionary
    data = {
        'positions': positions,
        'distance_matrix': distance_matrix,
        'demands': weights,
        'vehicle_capacities': [int(config.VEHICLE_CAPACITIES)],
        'num_vehicles': 1,
        'depot': 0,
        'forbidden_connections': forbidden_connections,
        'starts': [config.START_POINT_TSP],
        'ends': [0],
    }
    return data


def main():
    """
    Main function to load the solution data and create the data model.
    """
    # Load the solution data
    data = load_files.load_solution_data()
    # Create the data model
    data = create_data_model(data['positions'], data['weights'], data['rows'])
    print(data)
    return data


if __name__ == '__main__':
    # Call the main function
    main()
