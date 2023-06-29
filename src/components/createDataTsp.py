from src.components import euclidianDistance
from src.components import forbiddenConnections
from dotenv import load_dotenv
import os
load_dotenv('/src/steps/.env')  # Ruta absoluta


# Costo computacional O(1)
def create_data_model(positions, weights, rows):
    """Stores the data for the problem."""
    distance_matrix = euclidianDistance.compute_euclidean_distance_matrix(positions)
    forbidden_connections = forbiddenConnections.generate_forbidden_connections(rows)
    data = {
        'positions': positions,
        'distance_matrix': distance_matrix,
        'demands': weights,
        'vehicle_capacities': [int(os.getenv('VEHICLE_CAPACITIES'))],
        'num_vehicles': 1,
        'depot': 0,
        'forbidden_connections': forbidden_connections,
    }
    return data
