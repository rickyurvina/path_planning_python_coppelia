from scipy.interpolate import interp1d


def interpolate_path(path=[(0, 0), (2, 3), (5, 1), (8, 4), (10, 0)]):
    """
    Interpolate a given path to obtain coordinates at any given time.

    Args:
        path (list): List of tuples representing x, y coordinates of the path points. Default is [(0, 0), (2, 3), (5, 1), (8, 4), (10, 0)].

    Returns:
        function: A function that interpolates the path at any given time and returns the interpolated x and y coordinates.
    """
    # Assign time values to each point in the path
    rrt_path_with_time = [(x, y, time) for time, (x, y) in enumerate(path)]

    # Create interpolation functions
    x_values, y_values, time_values = zip(*rrt_path_with_time)
    interpolate_x = interp1d(time_values, x_values, kind='linear', fill_value='extrapolate')
    interpolate_y = interp1d(time_values, y_values, kind='linear', fill_value='extrapolate')

    # Return a single function that interpolates both x and y
    def path_interpolate(current_time):
        x = interpolate_x(current_time)
        y = interpolate_y(current_time)
        return x, y

    return path_interpolate


if __name__ == '__main__':
    # Usage of the function
    interpolated_path = interpolate_path()

    # Get coordinates at time t
    t = 2.5  # adjust this value according to the desired time
    x_t, y_t = interpolated_path(t)

    print(f"Coordinates at time {t}: ({x_t}, {y_t})")
