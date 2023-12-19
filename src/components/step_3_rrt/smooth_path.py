import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


def smooth_path(path, smoothness=0.1):
    """
    Smoothens the path using cubic splines.

    Parameters:
    - path: List of points on the path [(x1, y1), (x2, y2), ...]
    - smoothness: Parameter controlling the smoothness of the resulting trajectory.

    Returns:
    - List of smoothed points [(x1_smooth, y1_smooth), (x2_smooth, y2_smooth), ...]
    """
    path = np.array(path)
    t = np.arange(0, len(path), 1)

    # Apply cubic splines
    cs = CubicSpline(t, path, bc_type='clamped')

    # Interpolate to obtain a smoothed trajectory
    t_smooth = np.arange(0, len(path) - 1, smoothness)
    path_smooth = cs(t_smooth)

    return path_smooth


def plot_path(original_path, smoothed_path):
    """
    Plots the original path and the smoothed path.

    Parameters:
    - original_path: List of points on the original path [(x1, y1), (x2, y2), ...]
    - smoothed_path: List of points on the smoothed path [(x1_smooth, y1_smooth), (x2_smooth, y2_smooth), ...]
    """
    original_path = np.array(original_path)
    smoothed_path = np.array(smoothed_path)

    plt.figure(figsize=(8, 8))
    plt.plot(original_path[:, 0], original_path[:, 1], '-o', label='Original Path')
    plt.plot(smoothed_path[:, 0], smoothed_path[:, 1], '-o', label='Smoothed Path')

    plt.title('Path Smoothing with Cubic Splines')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    # Example of usage
    original_path = [(0, 0), (1, 1), (2, 4), (3, 1), (4, 0)]
    smoothed_path = smooth_path(original_path)

    # Plot the original path and the smoothed path
    plot_path(original_path, smoothed_path)
