
from src.coppelia import sim
import numpy as np
import cv2
import time
import os

def save_image(img):
    folder = "../images"
    prefix = "ambiente"
    ext = ".jpg"

    # Verificar la existencia de archivo y establecer el contador
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1

    # Guardar la imagen con el nombre adecuado
    filename = os.path.join(folder, prefix + str(i) + ext)
    cv2.imwrite(filename, img)
    print("Guardado como", filename)
# Conectar con Coppelia
sim.simxFinish(-1) # Cerrar todas las conexiones existentes
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Conectar con Coppelia
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
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # Convertir de RGB a BGR
    img = cv2.flip(img, 0)  # Voltear la imag
    # Mostrar la imagen en una ventana de OpenCV
    cv2.imshow('Imagen', img)
    save_image(img)
    cv2.waitKey(0)

# Cerrar la conexión con Coppelia
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(clientID)
print('Conexión cerrada')

