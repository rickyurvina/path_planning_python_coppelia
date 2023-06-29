import math

def compute_euclidean_distance_matrix(positions):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(positions):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(positions):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1])) * 100))
    return distances