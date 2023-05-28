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
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convertir de RGB a BGR

            # Convertir la imagen a escala de grises
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Binarizar la imagen utilizando un umbral adaptativo
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

            # Reducir el ruido de la imagen utilizando un filtro de mediana
            thresh = cv2.medianBlur(thresh, 5)

            # Obtener el occupancy grid a partir de la imagen binarizada
            occupancy_grid = (thresh > 0).astype(np.uint8)

            # Encontrar los contornos en el occupancy grid
            contours, hierarchy = cv2.findContours(occupancy_grid, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Crear una copia de la imagen original para dibujar los contornos
            img_contours = img.copy()

            # Recorrer todos los contornos encontrados
            for i in range(len(contours)):
                # Obtener el área del contorno
                area = cv2.contourArea(contours[i])

                # Ignorar los contornos pequeños
                if area < 1000:
                    continue

                # Obtener las coordenadas del rectángulo que encierra el contorno
                x, y, w, h = cv2.boundingRect(contours[i])

                # Dibujar el rectángulo en la imagen
                cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Transformar el mapa a coordenadas locales x e y
                mapa_local = np.zeros_like(occupancy_grid)
                mapa_local[occupancy_grid == 1] = 1000
                y, x = np.nonzero(mapa_local)
                x_local = x - mapa_local.shape[1] / 2
                y_local = -y + mapa_local.shape[0] / 2

                # Graficar el mapa con los contornos y los ejes
            plt.figure()
            plt.imshow(mapa_local, cmap='gray', origin='lower')
            plt.plot(0, 0, 'rx')
            # plt.plot(x_local, y_local, 'g.')
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
                'img_contours': img_contours,
                'mapa_local': mapa_local,
            }
            # Guardar las variables en un archivo pickle
            save_file(variables)

        def save_file(variables):
            folder = "../files"
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
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID)

generateOcuppancy()