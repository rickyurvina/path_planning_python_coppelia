from src.coppelia import sim
import numpy as np
import cv2
import time
from src.components import saveFiles
import matplotlib.pyplot as plt


def generate_rgb():
    # Conectar con Coppelia
    sim.simxFinish(-1)  # Cerrar todas las conexiones existentes
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conectar con Coppelia
    if clientID == -1:
        print('Error al conectar con Coppelia')
    else:
        print('Conectado con Coppelia')

    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    # Obtener el handle de la cámara
    res, v0 = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
    time.sleep(1)

    if res != 0:
        print('Error al obtener el handle de la cámara')
    else:
        print('Obtenido el handle de la cámara')

    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v0, 0, sim.simx_opmode_oneshot_wait)
    # Capturar una imagen de la cámara
    if err != 0:
        print('Error al obtener la imagen de la cámara')
    else:
        # Convertir la imagen en un array numpy
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # Convertir de RGB a BGR
        # img = cv2.flip(img, 0)  # Voltear la imag
        # plt.imshow(img)
        # plt.axis('off')
        # plt.show()
        # Mostrar la imagen en una ventana de OpenCV
        # cv2.imshow('Imagen', img)
        saveFiles.save_image(img)
        variables = {
            'prefix': "rgb",
            'img': img,
        }
        saveFiles.save_workspace(variables)

    # Cerrar la conexión con Coppelia
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID)
    print('Conexión cerrada')
    return img

# generate_rgb()