from src.coppelia import sim
import numpy as np
import cv2
import time
from src.components.plotsEnvironment import plot_occupancy


# Conectar con Coppelia
def generate_occupancy(clientID, object_handles, name_folder):
    try:
        # Obtener el handle del objeto al que se desea eliminar la textura
        _, box_terrain = sim.simxGetObjectHandle(clientID, 'box_terrain', sim.simx_opmode_blocking)
        # layer_property = sim.sim_objectproperty_cameravisibilitylayer
        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, False,
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
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Binarizar la imagen utilizando un umbral adaptativo
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

            # Reducir el ruido de la imagen utilizando un filtro de mediana
            thresh = cv2.medianBlur(thresh, 5)

            # Obtener el occupancy grid a partir de la imagen binarizada
            occupancy_grid = (thresh > 1).astype(np.float64)

        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, True,
                                            sim.simx_opmode_blocking)
        for i in range(len(object_handles)):
            res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                True,
                                                sim.simx_opmode_blocking)

        plot_occupancy(occupancy_grid, name_folder)

        return occupancy_grid

    except Exception as e:
        print(e)
