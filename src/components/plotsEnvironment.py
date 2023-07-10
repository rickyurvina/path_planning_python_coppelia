import matplotlib.pyplot as plt
from src.components import saveFiles
import cv2


def plot_occupancy(occupancy_grid, name_folder):
    plt.figure()
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.axis('equal')
    plt.savefig(saveFiles.get_name_to_save_plot(name_folder, 'occupancy_grid'))
    plt.show()


def plot_rgb(rgb, name_folder):
    # rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # Convertir de RGB a BGR
    # rgb = cv2.flip(rgb, 0)  # Voltear la imag
    plt.figure()
    plt.imshow(rgb, origin='lower', aspect='equal')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.axis('equal')
    plt.savefig(saveFiles.get_name_to_save_plot(name_folder, 'rgb'))
    plt.show()
