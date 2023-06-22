import matplotlib.pyplot as plt
from src.components import saveFiles
def plot_occupancy(occupancy_grid):
    plt.figure()
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Eje X (m)')
    plt.ylabel('Eje Y (m)')
    plt.axis('equal')
    plt.savefig(saveFiles.get_name_to_save_plot())
    plt.show()

