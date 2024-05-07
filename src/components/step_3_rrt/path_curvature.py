import numpy as np
from matplotlib import pyplot as plt
from src.components.step_3_rrt.path_to_x_y import get_path_coordinates


def calculate_curvature(x, y):
    """
    Calculate the curvature of a path defined by x and y coordinates.

    Args:
        x (array_like): Array of x coordinates.
        y (array_like): Array of y coordinates.

    Returns:
        tuple: Tuple containing curvature, second derivative of x, and second derivative of y.
    """
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvature = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt ** 2 + dy_dt ** 2) ** (3 / 2)
    return curvature, d2x_dt2, d2y_dt2


def calculate_curvature_variation(self, x=[], y=[]):
    """
    Calculate the variation of curvature along a path.

    Args:
        x (array_like): Array of x coordinates.
        y (array_like): Array of y coordinates.

    Returns:
        tuple: Tuple containing curvature and smoothness.
    """
    if len(x) == 0 or len(y) == 0:
        print("No path available")
    else:
        x, y = get_path_coordinates(self)
    path_x_y = np.column_stack((x, y))
    curvature, d2x_dt2, d2y_dt2 = calculate_curvature(x, y)
    smoothness = calculate_smoothness(d2x_dt2, d2y_dt2)
    return curvature, smoothness


def calculate_smoothness(d2x_dt2, d2y_dt2):
    """
    Calculate the smoothness of a path based on its second derivatives.

    Args:
        d2x_dt2 (array_like): Second derivative of x.
        d2y_dt2 (array_like): Second derivative of y.

    Returns:
        float: Smoothness value.
    """
    return np.mean(np.sqrt(d2x_dt2 ** 2 + d2y_dt2 ** 2))


def plot_curvature(curvature):
    """
    Plot the variation of curvature along a path.

    Args:
        curvature (array_like): Array of curvature values.
    """
    plt.figure(figsize=(10, 5))
    plt.plot(curvature, label='Curvature')
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.xlabel('Point on Path')
    plt.ylabel('Curvature')
    plt.title('Curvature Variation')
    plt.grid(True)
    plt.legend()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Suppose we have a list of coordinates representing a path
    # Here's a simple example of an S-shaped curve
    x = np.linspace(0, 10, 100)
    y = np.sin(x) + np.cos(x)

    # Calculate curvature variation
    curvature = calculate_curvature(x, y)
    variation = calculate_curvature_variation(x, y)
    print("Curvature Variation:", variation)

    # Plot the path
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, label='Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot the curvature variation
    plot_curvature(x, y)
