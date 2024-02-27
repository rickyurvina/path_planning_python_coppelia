from src.steps import config


def pixels_to_meters(path_array, resolution_px=config.RESOLUTION_X, length_m=config.MAP_WIDTH):
    pixels_per_meter = (resolution_px / length_m)
    path_meters = [(x / pixels_per_meter - 7.5, y / pixels_per_meter - 7.5) for x, y in path_array]

    return path_meters


if __name__ == '__main__':
    path_pixels = [(500, 500), (750, 750), (1000, 1000)]
    resolution_px = 1000
    length_m = 15

    path_meters = pixels_to_meters(path_pixels, resolution_px, length_m)
    print("Camino en metros:", path_meters)
