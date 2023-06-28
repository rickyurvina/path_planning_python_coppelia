from src.coppelia import sim
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from src.components.plotOcuppancyGrid import plot_occupancy
from src.components import saveFiles


# Conectar con Coppelia
def generateOcuppancy():
    try:
        sim.simxFinish(-1)  # Cerrar todas las conexiones existentes
        clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conectar con Coppelia
        if clientID == -1:
            print('Error al conectar con Coppelia')
        else:
            print('Conectado con Coppelia')

        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        # Obtener el handle del objeto al que se desea eliminar la textura
        _, box_terrain = sim.simxGetObjectHandle(clientID, 'box_terrain', sim.simx_opmode_blocking)
        # layer_property = sim.sim_objectproperty_cameravisibilitylayer
        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, False,
                                            sim.simx_opmode_blocking)
        # Obtener el handle del sensor de visión
        res, sensor_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
        # Obtener la transformación del sensor de visión con respecto al marco de coordenadas locales del mundo
        _, sensor_pos = sim.simxGetObjectPosition(clientID, sensor_handle, -1, sim.simx_opmode_oneshot_wait)
        _, sensor_ori = sim.simxGetObjectOrientation(clientID, sensor_handle, -1, sim.simx_opmode_oneshot_wait)

        time.sleep(1)

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

            mapa_local = np.zeros_like(occupancy_grid)
            mapa_local[occupancy_grid == 1] = 1

        res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, True,
                                            sim.simx_opmode_blocking)

        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID)

        print('Conexión cerrada')
        variables = {
            'prefix': 'map',
            'img': img,
            'thresh': thresh,
            'occupancy_grid': occupancy_grid,
            'mapa_local': mapa_local,
        }
        saveFiles.save_workspace(variables)
        plot_occupancy(occupancy_grid)

        return occupancy_grid


    except Exception as e:
        print(e)
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID)


# generateOcuppancy()
