from src.components.step_3_rrt.distance import distance


def path_cost_final_smoothed(smoothed_path):
    total_cost = 0
    for i in range(len(smoothed_path) - 1):
        total_cost += distance(smoothed_path[i], smoothed_path[i + 1])
    return total_cost
