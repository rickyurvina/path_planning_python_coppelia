import traceback

from colorama import Fore
from matplotlib import pyplot as plt
from src.coppelia import sim
from src.coppelia import sim
import numpy as np
import cv2
from src.components.plotsEnvironment import plot_occupancy
from src.components import createFolder
from src.components import getPositionsObjects
import generateRGB
import generateOccupancyGrid


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
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # img = cv2.flip(img, 0)  # Voltear la imagen

            # Binarizar la imagen utilizando un umbral adaptativo
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 111, 2)

            res = sim.simxSetObjectIntParameter(clientID, box_terrain, sim.sim_objintparam_visibility_layer, True,
                                                sim.simx_opmode_blocking)
            for i in range(len(object_handles)):
                res = sim.simxSetObjectIntParameter(clientID, object_handles[i], sim.sim_objintparam_visibility_layer,
                                                    True,
                                                    sim.simx_opmode_blocking)

                # Recorrer todos los contornos encontrados
            occupancy_grid = (thresh > 1).astype(np.uint8)
            contours, hierarchy = cv2.findContours(occupancy_grid, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            img_contours = img.copy()

            for i in range(len(contours)):
                # Obtener el área del contorno
                area = cv2.contourArea(contours[i])

                # Ignorar los contornos pequeños
                if area < 100 or area>40000:
                    continue

                # Obtener las coordenadas del rectángulo que encierra el contorno
                x, y, w, h = cv2.boundingRect(contours[i])
                occupancy_grid[y:y + h, x:x + w] = 0
                # Dibujar el rectángulo en la imagen
                cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # plt.imshow(img_contours)
            # plt.show()

        return occupancy_grid

    except Exception as e:
        print(e)

def startSimulation():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    try:
        if clientID == -1:
            print('No se pudo conectar con CoppeliaSim')
            exit()
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
        return clientID
    except Exception as e:
        print(e)
        if clientID != -1:
            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
            sim.simxFinish(clientID)
def closeSimulation(clientID):
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)


def draw_map(occupancy_grid,  name='Occupancy Grid'):
    try:
        fig, ax = plt.subplots(1)
        ax.imshow(occupancy_grid, cmap='gray', origin='lower')
        plt.title(name)
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')

        plt.show()
    except Exception as e:
        print(Fore.RED + e)
        traceback.print_exc()

if __name__ == '__main__':

    clientId = startSimulation()
    positions, weights, rows, object_handles = getPositionsObjects.get_positions(clientId)
    name_folder = createFolder.create_folder()
    occupancy_grid=generate_occupancy(clientId, object_handles, name_folder)
    draw_map(occupancy_grid)
    closeSimulation(clientId)
