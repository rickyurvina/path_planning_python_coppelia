from src.steps import config


def pixels_to_meters(path_array, resolution_px=config.RESOLUTION_X, length_m=config.MAP_WIDTH):
    """
    Convert a path from pixel coordinates to meter coordinates.

    Args:
        path_array (list): List of tuples representing x, y coordinates of the path points in pixels.
        resolution_px (int): Resolution of the map in pixels. Defaults to config.RESOLUTION_X.
        length_m (int): Length of the map in meters. Defaults to config.MAP_WIDTH.

    Returns:
        list: List of tuples representing x, y coordinates of the path points in meters.
    """
    pixels_per_meter = (resolution_px / length_m)
    path_meters = [(x / pixels_per_meter - length_m / 2, y / pixels_per_meter - length_m / 2) for x, y in path_array]

    return path_meters


if __name__ == '__main__':
    path_pixels = [(500, 500), (750, 750), (1000, 1000)]
    resolution_px = 1000
    length_m = 15

    path_meters = pixels_to_meters(path_pixels, resolution_px, length_m)
    print("Path in meters:", path_meters)
