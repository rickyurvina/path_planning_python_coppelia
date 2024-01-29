from matplotlib import pyplot as plt

from src.components.step1_get_data.close_simulation_coppelia import close_simulation
from src.components.common.draw_maps import draw_maps
from src.components.step1_get_data.get_positions_objects import get_positions_of_objects_to_hide
from src.components.step1_get_data.start_simulation_coppelia import startSimulation
from src.components.step1_get_data.traversability import can_traverse_terrain
from src.coppelia import sim
import numpy as np
import cv2
from src.components import create_folder
from src.components.step1_get_data import get_positions_objects
from src.components.step1_get_data.generate_rgb import generate_rgb
from src.steps import config


def generate_occupancy_grid(clientID, object_handles):
    try:
        # Obtener el handle del objeto al que se desea eliminar la textura
        _, box_terrain = sim.simxGetObjectHandle(clientID, 'box_terrain', sim.simx_opmode_blocking)
        _, husky = sim.simxGetObjectHandle(clientID, 'Husky', sim.simx_opmode_blocking)
        # layer_property = sim.sim_objectproperty_cameravisibilitylayer
        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, False,
                                            sim.simx_opmode_blocking)
        res = sim.simxSetObjectIntParameter(clientID, husky, sim.sim_objintparam_visibility_layer, False,
                                            sim.simx_opmode_blocking)

        for i in range(len(object_handles)):
            res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                False,
                                                sim.simx_opmode_blocking)

        # Obtener el handle del sensor de visión
        res, sensor_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
        # Obtener la transformación del sensor de visión con respecto al marco de coordenadas locales del mundo
        _, sensor_pos = sim.simxGetObjectPosition(clientID, sensor_handle, -1, sim.simx_opmode_oneshot_wait)
        _, sensor_ori = sim.simxGetObjectOrientation(clientID, sensor_handle, -1, sim.simx_opmode_oneshot_wait)

        if res != 0:
            print('Error al obtener el handle del sensor de visión')
        else:
            print('Obtenido el handle del sensor de visión')

        # Obtener la imagen capturada por el sensor de visión
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, sensor_handle, 0, sim.simx_opmode_oneshot_wait)

        if err != 0:
            print('Error al obtener la imagen del sensor de visión')
        else:
            # Convertir la imagen en un array numpy
            img_arr = np.array(image, dtype=np.uint8)
            img_arr.resize([resolution[1], resolution[0], 3])

            gray = cv2.cvtColor(img_arr, cv2.COLOR_BGR2GRAY)
            img = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)

            # Binarizar la imagen utilizando un umbral adaptativo
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

            res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, True,
                                                sim.simx_opmode_blocking)
            res = sim.simxSetObjectIntParameter(clientID, husky, sim.sim_objintparam_visibility_layer, True,
                                                sim.simx_opmode_blocking)
            for i in range(len(object_handles)):
                res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                    True,
                                                    sim.simx_opmode_blocking)

                # Recorrer todos los contornos encontrados
            occupancy_grid = (thresh > 1).astype(np.uint8)

        return occupancy_grid, gray, img

    except Exception as e:
        print(e)


def generate_occupancy_processed(gray):
    try:
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 111, 2)
        occupancy_grid = (thresh > 1).astype(np.uint8)
        return occupancy_grid

    except Exception as e:
        print(e)


def generate_occupancy_grid_filled(img, occupancy_grid):
    contours, hierarchy = cv2.findContours(occupancy_grid, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_contours = img.copy()

    for i in range(len(contours)):
        # Obtener el área del contorno
        area = cv2.contourArea(contours[i])

        # Ignorar los contornos pequeños
        if area > 40000:
            continue

        # Obtener las coordenadas del rectángulo que encierra el contorno
        x, y, w, h = cv2.boundingRect(contours[i])
        occupancy_grid[y:y + h, x:x + w] = 0
        # Dibujar el rectángulo en la imagen
        cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)
    occupancy_grid_filled = occupancy_grid.copy()
    return img_contours, occupancy_grid_filled


def generate_occupancy_grid_filled_terrain(img, list_occupancy_grid):
    list_occupancy_grid_filled = []
    frictions_values = [0.1, 0.3, 0.7, 0.9, 0.2]
    cont = 0
    rows, cols = config.RESOLUTION_X, config.RESOLUTION_Y
    for occupancy_grid in list_occupancy_grid:
        # occupancy_grid = cv2.flip(occupancy_grid, 0)
        contours, hierarchy = cv2.findContours(occupancy_grid, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        img_contours = img.copy()
        # img_contours = cv2.flip(img_contours, 0)  # Voltear la imagen

        occupancy_grid_to_export = np.ones((rows, cols))

        for i in range(len(contours)):
            # Obtener el área del contorno
            area = cv2.contourArea(contours[i])

            # Ignorar los contornos pequeños
            if area > 40000:
                continue

            # Obtener las coordenadas del rectángulo que encierra el contorno
            x, y, w, h = cv2.boundingRect(contours[i])
            occupancy_grid_to_export[y:y + h, x:x + w] = frictions_values[cont]
            # Dibujar el rectángulo en la imagen
            # cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)
            list_occupancy_grid_filled.append(occupancy_grid_to_export)
        cont += 1

    merged_occupancy_grid = combine_occupancy_grids_terrain(list_occupancy_grid_filled)
    return img_contours, merged_occupancy_grid


def generate_og_traversability(clientID, object_handles, terrain_handles, frictions_values):
    try:
        list_occupancy_grid = []
        # Obtener el handle del objeto al que se desea eliminar la textura
        _, box_terrain = sim.simxGetObjectHandle(clientID, 'box_terrain', sim.simx_opmode_blocking)
        _, husky = sim.simxGetObjectHandle(clientID, 'Husky', sim.simx_opmode_blocking)
        # layer_property = sim.sim_objectproperty_cameravisibilitylayer
        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, False,
                                            sim.simx_opmode_blocking)
        res = sim.simxSetObjectIntParameter(clientID, husky, sim.sim_objintparam_visibility_layer, False,
                                            sim.simx_opmode_blocking)
        for i in range(len(object_handles)):
            res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                False,
                                                sim.simx_opmode_blocking)
        for i in range(len(terrain_handles)):
            res = sim.simxSetObjectIntParameter(clientID, terrain_handles[i],
                                                sim.sim_objintparam_visibility_layer,
                                                False,
                                                sim.simx_opmode_blocking)
        for i in range(len(terrain_handles)):
            res = sim.simxSetObjectIntParameter(clientID, terrain_handles[i],
                                                sim.sim_objintparam_visibility_layer,
                                                True,
                                                sim.simx_opmode_blocking)
            for j in range(len(terrain_handles)):
                if j != i:
                    res = sim.simxSetObjectIntParameter(clientID, terrain_handles[j],
                                                        sim.sim_objintparam_visibility_layer,
                                                        False,
                                                        sim.simx_opmode_blocking)

            # frictions_values = [0.3, 0.4]
            res, sensor_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
            err, resolution, image = sim.simxGetVisionSensorImage(clientID, sensor_handle, 0,
                                                                  sim.simx_opmode_oneshot_wait)

            # Convertir la imagen en un array numpy
            img_arr = np.array(image, dtype=np.uint8)
            img_arr.resize([resolution[1], resolution[0], 3])

            gray = cv2.cvtColor(img_arr, cv2.COLOR_BGR2GRAY)
            img = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)

            # Binarizar la imagen utilizando un umbral adaptativo
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
            occupancy_grid = (thresh > 1).astype(np.uint8)
            list_occupancy_grid.append(occupancy_grid)

        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, True,
                                            sim.simx_opmode_blocking)
        res = sim.simxSetObjectIntParameter(clientID, husky, sim.sim_objintparam_visibility_layer, True,
                                            sim.simx_opmode_blocking)
        for i in range(len(object_handles)):
            res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                True,
                                                sim.simx_opmode_blocking)

        for i in range(len(terrain_handles)):
            res = sim.simxSetObjectIntParameter(clientID, terrain_handles[i], sim.sim_objintparam_visibility_layer,
                                                True,
                                                sim.simx_opmode_blocking)

        return img, list_occupancy_grid
    except Exception as e:
        print(e)


def combine_occupancy_grids(occupancy_grid_filled, occupancy_grid_filled_traver):
    list_occupancy_grid_ = [occupancy_grid_filled, occupancy_grid_filled_traver]
    merged_occupancy_grid = np.ones_like(occupancy_grid_filled)

    for occupancy_grid in list_occupancy_grid_:
        merged_occupancy_grid = np.minimum(merged_occupancy_grid, occupancy_grid)
    # combined = occupancy_grid + occupancy_grid_traver
    # grid_size = (config.RESOLUTION_X, config.RESOLUTION_Y)
    # # combined = np.clip(combined, 0, 1)
    # for i in range(grid_size[0]):
    #     for j in range(grid_size[1]):
    #         if combined[i, j] == 1:
    #             combined[i, j] = 0
    #         elif combined[i, j] == 2:
    #             combined[i, j] = 1
    #         elif combined[i, j] == 1.3:
    #             combined[i, j] = 0.3
    return merged_occupancy_grid


def combine_occupancy_grids_terrain(list_occupancy_grid):
    merged_occupancy_grid = np.ones_like(list_occupancy_grid[0])

    for occupancy_grid in list_occupancy_grid:
        merged_occupancy_grid = np.minimum(merged_occupancy_grid, occupancy_grid)
    plt.imshow(merged_occupancy_grid, cmap='gray', vmin=0, vmax=1)
    plt.colorbar()
    plt.title('Low traction terrain')
    plt.show()
    return merged_occupancy_grid


def main(clientId, object_handles):
    object_handles_traver, terrain_handles, frictions_values = get_positions_of_objects_to_hide(clientId)
    array_objects = np.concatenate((object_handles, terrain_handles))
    occupancy_grid, gray, img = generate_occupancy_grid(clientId, array_objects)
    occupancy_grid_processed = generate_occupancy_processed(gray)
    occupancy_grid_processed_copy = occupancy_grid_processed.copy()
    img_contours, occupancy_grid_filled = generate_occupancy_grid_filled(img, occupancy_grid_processed_copy)
    if config.DETECT_FRICTION_TERRAIN:
        img_traver, list_occupancy_grid = generate_og_traversability(clientId, object_handles_traver, terrain_handles,
                                                                     frictions_values)
        img_contours_tra, occupancy_grid_filled_traver = generate_occupancy_grid_filled_terrain(img_traver,
                                                                                                list_occupancy_grid)
        occupancy_grid_combined = combine_occupancy_grids(occupancy_grid_filled, occupancy_grid_filled_traver)

        return occupancy_grid_combined
    else:
        return occupancy_grid_filled


if __name__ == '__main__':
    clientId = startSimulation()
    _, _, _, object_handles = get_positions_objects.get_positions(clientId)
    name_folder = create_folder.create_folder("../../solutions")
    object_handles_traver, terrain_handles, frictions_values = get_positions_of_objects_to_hide(clientId)
    array_objects = np.concatenate((object_handles, terrain_handles))
    occupancy_grid, gray, img = generate_occupancy_grid(clientId, array_objects)
    rbg = generate_rgb(clientId, name_folder)
    occupancy_grid_processed = generate_occupancy_processed(gray)
    occupancy_grid_processed_copy = occupancy_grid_processed.copy()
    img_contours, occupancy_grid_filled = generate_occupancy_grid_filled(img, occupancy_grid_processed_copy)
    img_traver, list_occupancy_grid = generate_og_traversability(clientId, object_handles_traver, terrain_handles,
                                                                 frictions_values)
    img_contours_tra, occupancy_grid_filled_traver = generate_occupancy_grid_filled_terrain(img_traver,
                                                                                            list_occupancy_grid)
    occupancy_grid_combined = combine_occupancy_grids(occupancy_grid_filled, occupancy_grid_filled_traver)

    map_data = [
        (rbg, 'RGB'),
        (occupancy_grid, 'Occupancy Grid (OG)'),
        (occupancy_grid_processed, 'Expanded OG'),
        (img_contours, 'OG Contours'),
        (occupancy_grid_filled_traver, 'Low traction terrain'),
        (occupancy_grid_combined, 'OG Combined'),
    ]
    draw_maps(map_data, name_folder)
    close_simulation(clientId)
