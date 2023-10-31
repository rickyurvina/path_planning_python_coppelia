import traceback

from colorama import Fore
from matplotlib import pyplot as plt


def draw_map(occupancy_grid, name='Occupancy Grid'):
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
