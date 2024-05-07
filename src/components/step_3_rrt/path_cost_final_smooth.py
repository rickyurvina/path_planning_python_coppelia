from src.components.step_3_rrt.distance import distance


def path_cost_final_smoothed(smoothed_path):
    """
    Calculate the final cost of a smoothed path.

    Args:
        smoothed_path (list): List of Node objects representing the smoothed path.

    Returns:
        float: Final cost of the smoothed path.
    """
    total_cost = 0
    for i in range(len(smoothed_path) - 1):
        total_cost += distance(smoothed_path[i], smoothed_path[i + 1])
    return total_cost
