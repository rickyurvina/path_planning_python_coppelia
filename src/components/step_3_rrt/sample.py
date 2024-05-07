from src.components.step_3_rrt.get_new_point import get_new_point
from src.components.step_3_rrt.get_new_point_in_ellipsoid import get_new_point_in_ellipsoid


def sample(self, start, goal, goal_bias=0.05, c_best=0):
    """
    Generate a new sample point for the RRT.

    Args:
        self: Instance of the RRT class.
        start: Start point for sampling.
        goal: Goal point for sampling.
        goal_bias: Bias for sampling towards the goal.
        c_best: Cost associated with the best path.

    Returns:
        Tuple: New sample point.
    """
    self.total_samples += 1
    if c_best <= 0:
        new_point = get_new_point(self, goal_bias, start, goal)
    else:
        new_point = get_new_point_in_ellipsoid(self, goal_bias, c_best, start, goal)
    return new_point
