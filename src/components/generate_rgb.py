from src.components.close_simulation_coppelia import close_simulation
from src.components.start_simulation_coppelia import startSimulation
from src.coppelia import sim
import numpy as np
import time
from src.components import save_files, create_folder
from src.components.plots_environment import plot_rgb


def generate_rgb(clientID, name_folder):
    try:
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
            # plot_rgb(img, name_folder)
    except Exception as e:
        print(e)

    return img


def main():
    name_folder = create_folder.create_folder()
    clientID = startSimulation()
    rgb = generate_rgb(clientID, name_folder)
    close_simulation(clientID)


if __name__ == '__main__':
    main()
