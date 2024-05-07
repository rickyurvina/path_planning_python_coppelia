import numpy as np
import matplotlib.pyplot as plt


def calculate_second_derivative(x, y, t):
    """
    Calculate the second derivative of x and y with respect to time.

    Args:
        x (numpy.ndarray): Array of x-coordinates.
        y (numpy.ndarray): Array of y-coordinates.
        t (numpy.ndarray): Array of time values.

    Returns:
        numpy.ndarray: Second derivative of x with respect to time.
        numpy.ndarray: Second derivative of y with respect to time.
    """
    dx_dt = np.gradient(x, t)
    dy_dt = np.gradient(y, t)
    d2x_dt2 = np.gradient(dx_dt, t)
    d2y_dt2 = np.gradient(dy_dt, t)
    return d2x_dt2, d2y_dt2


def calculate_smoothness(d2x_dt2, d2y_dt2):
    """
    Calculate the smoothness of a path using the second derivatives.

    Args:
        d2x_dt2 (numpy.ndarray): Second derivative of x with respect to time.
        d2y_dt2 (numpy.ndarray): Second derivative of y with respect to time.

    Returns:
        float: Smoothness of the path.
    """
    return np.mean(np.sqrt(d2x_dt2 ** 2 + d2y_dt2 ** 2))


def generate_random_path(t):
    """
    Generate a random path.

    Args:
        t (numpy.ndarray): Array of time values.

    Returns:
        numpy.ndarray: Array of x-coordinates for the random path.
        numpy.ndarray: Array of y-coordinates for the random path.
    """
    np.random.seed(0)
    x = np.cumsum(np.random.randn(len(t)))
    y = np.cumsum(np.random.randn(len(t)))
    return x, y


# Example usage
if __name__ == "__main__":
    # Define time
    t = np.linspace(0, 10, 100)

    # Generate sinusoidal and random paths
    x1, y1 = np.sin(t), np.cos(t)
    x2, y2 = generate_random_path(t)

    # Calculate second derivatives for each path
    d2x_dt2_1, d2y_dt2_1 = calculate_second_derivative(x1, y1, t)
    d2x_dt2_2, d2y_dt2_2 = calculate_second_derivative(x2, y2, t)

    # Calculate smoothness for each path
    smoothness_1 = calculate_smoothness(d2x_dt2_1, d2y_dt2_1)
    smoothness_2 = calculate_smoothness(d2x_dt2_2, d2y_dt2_2)

    # Print smoothness for each path
    print("Smoothness of sinusoidal path:", smoothness_1)
    print("Smoothness of random path:", smoothness_2)

    # Plot paths
    plt.figure(figsize=(10, 5))
    plt.plot(x1, y1, label='Sinusoidal Path')
    plt.plot(x2, y2, label='Random Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path Comparison')
    plt.grid(True)
    plt.legend()
    plt.show()
