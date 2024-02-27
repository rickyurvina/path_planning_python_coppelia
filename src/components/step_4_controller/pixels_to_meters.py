from src.steps import config


def pixels_to_meters(path_array, resolution_px=config.RESOLUTION_X, length_m=config.MAP_WIDTH):
    # Calcular la relación píxeles por metro
    pixels_per_meter = (resolution_px / length_m)

    # Convertir las coordenadas de píxeles a metros
    path_meters = [(x / pixels_per_meter - 7.5, y / pixels_per_meter - 7.5) for x, y in path_array]

    return path_meters


if __name__ == '__main__':
    # Ejemplo de uso
    path_pixels = [(500, 500), (750, 750), (1000, 1000)]  # Ejemplo de camino en píxeles
    resolution_px = 1000  # Resolución del mapa en píxeles
    length_m = 15  # Longitud del mapa en metros

    path_meters = pixels_to_meters(path_pixels, resolution_px, length_m)
    print("Camino en metros:", path_meters)
