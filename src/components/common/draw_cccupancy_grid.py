import traceback

from colorama import Fore
from matplotlib import pyplot as plt


def draw_map(occupancy_grid, name='Occupancy Grid'):
    """
    Draw the occupancy grid map using matplotlib.
    """
    try:
        # Create a new figure and a set of subplots
        fig, ax = plt.subplots(1)
        # Display an image on the axes
        ax.imshow(occupancy_grid, cmap='gray', origin='lower')
        # Set a title for the axes
        plt.title(name)
        # Place a legend on the axes
        plt.legend()
        # Set the label for the x-axis
        plt.xlabel('X')
        # Set the label for the y-axis
        plt.ylabel('Y')

        # Display the figure
        plt.show()
    except Exception as e:
        # Print the error message in red
        print(Fore.RED + str(e))
        # Print the full traceback for the exception
        traceback.print_exc()
