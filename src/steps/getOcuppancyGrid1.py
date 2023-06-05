from src.coppelia import sim
import numpy as np
import cv2
import time

try:
    print("Running")

    # Conectar con Coppelia
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
        img = cv2.flip(img, 0) # Voltear la imagen
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convertir de RGB a BGR

        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Binarizar la imagen utilizando un umbral adaptativo
        thresh = cv2.adaptiveThreshold(gray, 1000, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        # Reducir el ruido de la imagen utilizando un filtro de mediana
        thresh = cv2.medianBlur(thresh, 5)

        # Obtener el occupancy grid a partir de la imagen binarizada
        occupancy_grid = (thresh > 0).astype(np.uint8)

        # Mostrar el occupancy grid en una ventana de OpenCV
        cv2.imshow('Occupancy grid', occupancy_grid * 255)
        cv2.waitKey(0)

    # Cerrar la conexión con Coppelia
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)
    print('Conexión cerrada')

except Exception as e:
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    print("Error detectado. La simulación ha sido detenida.")