import matplotlib.pyplot as plt
from src.components import save_files


def plot_occupancy(occupancy_grid, name_folder):
    plt.figure()
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Coordinates-X (px)')
    plt.ylabel('Coordinates-Y (px)')
    plt.axis('equal')
    plt.savefig(save_files.get_name_to_save_plot(name_folder, 'occupancy_grid'))
    plt.show()


def plot_rgb(rgb, name_folder):
    plt.figure()
    plt.imshow(rgb, origin='lower', aspect='equal')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Coordinates-X (px)')
    plt.ylabel('Coordinates-Y (px)')
    plt.axis('equal')
    plt.savefig(save_files.get_name_to_save_plot(name_folder, 'rgb'))
    plt.show()
