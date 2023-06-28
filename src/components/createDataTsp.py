from src.components import objectsCoppelia


# Costo computacional O(1)
def create_data_model(num_objects, num_rows):
    """Stores the data for the problem."""
    distance_matrix, weights, positions, forbidden_connections = objectsCoppelia.get_data(
        num_objects, num_rows)
    data = {
        'positions': positions,
        'distance_matrix': distance_matrix,
        'demands': weights,
        'vehicle_capacities': [45],
        'num_vehicles': 1,
        'depot': 0,
        'forbidden_connections': forbidden_connections,
    }
    return data
