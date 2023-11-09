import traceback
from colorama import Fore
from matplotlib import pyplot as plt

from src.components.common import save_files


def draw_maps(map_data, name_folder):
    try:
        num_maps = len(map_data)

        # Configura una figura con subtramas en una fila
        fig, axes = plt.subplots(1, num_maps, figsize=(15, 5))

        # Itera a través de las imágenes y nombres
        for i, (occupancy_grid, name) in enumerate(map_data):
            ax = axes[i]
            if name == 'RGB':
                ax.imshow(occupancy_grid, origin='lower', aspect='equal')
            else:
                ax.imshow(occupancy_grid, cmap='gray', origin='lower')
            ax.set_title(name)
            ax.set_xlabel('X-Coordinates (px)', fontsize=14)
            ax.set_ylabel('Y-Coordinates (px)', fontsize=14)

        # Ajusta los márgenes entre las subtramas
        plt.tight_layout()
        # TODO fix name folder
        plt.savefig(save_files.get_name_to_save_plot(name_folder, 'image_process', '../../solutions'))
        plt.show()


    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()
