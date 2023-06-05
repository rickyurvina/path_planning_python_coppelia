from src.coppelia import sim
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import pickle
import os

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

            plt.figure()
            plt.imshow(occupancy_grid, cmap='gray', origin='lower')
            # plt.imshow(mapa_local, cmap='gray', origin='lower')
            plt.plot(0, 0, 'rx')
            plt.xlabel('Eje X (m)')
            plt.ylabel('Eje Y (m)')
            plt.axis('equal')
            plt.show()

        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID)

        print('Conexión cerrada')

        def save_workspace():
            # Crear un diccionario con todas las variables del programa
            variables = {
                'img': img,
                'thresh': thresh,
                'occupancy_grid': occupancy_grid,
                'mapa_local': mapa_local,
            }
            # Guardar las variables en un archivo pickle
            save_file(variables)

        def save_file(variables):
            folder = "../files/workspaces"
            prefix = "variables_map"
            ext = ".pickle"

            # Verificar la existencia de archivo y establecer el contador
            i = 1
            while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
                i += 1

            # Guardar la imagen con el nombre adecuado
            filename = os.path.join(folder, prefix + str(i) + ext)

            with open(filename, 'wb') as f:
                pickle.dump(variables, f)
            print('Variables guardadas en el archivo "variables_map.pickle"')

        save_workspace()
    except Exception as e:
        print(e)
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID)

generateOcuppancy()
