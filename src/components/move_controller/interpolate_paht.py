from scipy.interpolate import interp1d


def interpolate_path(path=[(0, 0), (2, 3), (5, 1), (8, 4), (10, 0)]):
    # Assign time values to each point in the path
    rrt_path_with_time = [(x, y, time) for time, (x, y) in enumerate(path)]

    print(rrt_path_with_time)

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
    # Uso de la función
    interpolated_path = interpolate_path()

    # Obtener coordenadas en el tiempo t
    t = 2.5  # ajusta este valor según el tiempo deseado
    x_t, y_t = interpolated_path(t)

    print(f"Coordenadas en el tiempo {t}: ({x_t}, {y_t})")
