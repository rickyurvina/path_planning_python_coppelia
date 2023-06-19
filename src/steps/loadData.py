from src.components import displayGraphW, getDynamicObjectsCoppelia, displayGraph

num_objects = 5
distance_matrix, weights, positions = getDynamicObjectsCoppelia.getData(num_objects)


# Costo computacional O(1)
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['positions'] = positions
    data['distance_matrix'] = distance_matrix
    data['demands'] = weights
    data['vehicle_capacities'] = [180]
    data['num_vehicles'] = 1
    data['depot'] = 0
    data['forbidden_connections'] = [(3, 4), (2, 4)]
    data['starts'] = [0]
    data['ends'] = [2]
    return data
