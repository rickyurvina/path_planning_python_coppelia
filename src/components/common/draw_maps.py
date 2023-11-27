import traceback
from colorama import Fore
from matplotlib import pyplot as plt

from src.components.common import save_files


def draw_maps(map_data, name_folder):
    try:
        num_maps = len(map_data)

        # Configura una figura con subtramas en una fila
        fig, axes = plt.subplots(2, 3, figsize=(15, 5))

        # Itera a través de las imágenes y nombres
        for i, (occupancy_grid, name) in enumerate(map_data):
            row = i // 3  # Calcula la fila actual
            col = i % 3  # Calcula la columna actual
            ax = axes[row, col]
            if name == 'RGB':
                ax.imshow(occupancy_grid, origin='lower', aspect='equal')
            else:
                im = ax.imshow(occupancy_grid, cmap='gray', vmin=0, vmax=1)
                if row == 1 and col == 1:
                    cbar = fig.colorbar(im, ax=ax)
                    cbar.set_label('Ground Friction', rotation=270, labelpad=15)
                    # cbar.invert_yaxis()

                    # ax.colorbar()
            ax.set_title(name)
            ax.set_xlabel('X-Coordinates (px)', fontsize=12)
            ax.set_ylabel('Y-Coordinates (px)', fontsize=12)

        plt.tight_layout()
        plt.savefig(save_files.get_name_to_save_plot(name_folder, 'image_process', '../../solutions', '.svg'),
                    format='svg')
        plt.show()


    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()
