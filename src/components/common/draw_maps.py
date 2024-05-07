import traceback
from colorama import Fore
from matplotlib import pyplot as plt

from src.components.common import save_files
from src.steps import config


def draw_maps(map_data, name_folder):
    """
    Draw maps from the provided map data and save the plot to a specified folder.
    """
    try:
        num_maps = len(map_data)

        # Configure a figure with subplots in a row
        fig, axes = plt.subplots(2, 3, figsize=(15, 5))

        # Iterate through the images and names
        for i, (occupancy_grid, name) in enumerate(map_data):
            row = i // 3  # Calculate the current row
            col = i % 3  # Calculate the current column
            ax = axes[row, col]
            if name == 'RGB':
                ax.imshow(occupancy_grid, origin='lower', aspect='equal')
            else:
                im = ax.imshow(occupancy_grid, cmap='gray', vmin=0, vmax=1)
                if row == 1 and col == 1:
                    cbar = fig.colorbar(im, ax=ax)
                    cbar.set_label('Ground Friction', rotation=270, labelpad=15)

            ax.set_title(name)
            ax.set_xlabel('X-Coordinates (px)', fontsize=10)
            ax.set_ylabel('Y-Coordinates (px)', fontsize=10)

        plt.tight_layout()
        plt.savefig(save_files.get_name_to_save_plot(name_folder, 'image_process', config.PATH_FOLDER, '.svg'),
                    format='svg')
        plt.show()

    except Exception as e:
        # Print the error message in red
        print(Fore.RED + str(e))
        # Print the full traceback for the exception
        traceback.print_exc()
