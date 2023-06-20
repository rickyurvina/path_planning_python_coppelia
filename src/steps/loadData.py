from src.components import displayGraphW, getDynamicObjectsCoppelia, displayGraph, \
    getDynamicObjectsCoppeliaWithConstraints

# distance_matrix, weights, positions = getDynamicObjectsCoppelia.getData(num_objects)

# Costo computacional O(1)
def create_data_model(num_objects, num_rows):
    """Stores the data for the problem."""
    distance_matrix, weights, positions, forbidden_connections = getDynamicObjectsCoppeliaWithConstraints.get_data(
        num_objects, num_rows)
    data = {}
    data['positions'] = positions
    data['distance_matrix'] = distance_matrix
    data['demands'] = weights
    data['vehicle_capacities'] = [100]
    data['num_vehicles'] = 1
    data['depot'] = 2
    data['forbidden_connections'] = forbidden_connections
    data['starts'] = [0]
    data['ends'] = [2]
    return data
