import matplotlib.pyplot as plt
from src.components.common import save_files


def plot_occupancy(occupancy_grid, name_folder):
    """
    Function to plot an occupancy grid.

    Parameters:
    occupancy_grid (2D array): The occupancy grid to be plotted.
    name_folder (str): The name of the folder where the plot will be saved.

    Returns:
    None
    """
    # Create a new figure
    plt.figure()

    # Display the occupancy grid as an image
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')

    # Mark the origin with a red cross
    plt.plot(0, 0, 'rx')

    # Set the labels for the x and y axes
    plt.xlabel('Coordinates-X (px)')
    plt.ylabel('Coordinates-Y (px)')

    # Set the aspect of the plot to be equal
    plt.axis('equal')

    # Save the plot to the specified folder
    plt.savefig(save_files.get_name_to_save_plot(name_folder, 'occupancy_grid'))

    # Display the plot
    plt.show()


def plot_rgb(rgb, name_folder):
    """
    Function to plot an RGB image.

    Parameters:
    rgb (3D array): The RGB image to be plotted.
    name_folder (str): The name of the folder where the plot will be saved.

    Returns:
    None
    """
    # Create a new figure
    plt.figure()

    # Display the RGB image
    plt.imshow(rgb, origin='lower', aspect='equal')

    # Mark the origin with a red cross
    plt.plot(0, 0, 'rx')

    # Set the labels for the x and y axes
    plt.xlabel('Coordinates-X (px)')
    plt.ylabel('Coordinates-Y (px)')

    # Set the aspect of the plot to be equal
    plt.axis('equal')

    # Save the plot to the specified folder
    plt.savefig(save_files.get_name_to_save_plot(name_folder, 'rgb'))

    # Display the plot
    plt.show()
