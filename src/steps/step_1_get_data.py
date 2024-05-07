# Import necessary modules and configurations
from src.steps import config

# If the application is set to online mode, import necessary components
if config.ON_LINE:
    from src.components.step1_get_data.close_simulation_coppelia import close_simulation
    from src.components.step1_get_data.start_simulation_coppelia import startSimulation

# Import necessary components
from src.components.step1_get_data import generate_cccupancy_grid, generate_rgb, get_positions_objects
from src.components import create_folder


def get_data(name_folder):
    """
    Function to get data from the simulation.
    Args:
        name_folder (str): The name of the folder where the data will be stored.
    Returns:
        dict: A dictionary containing the weights, positions, rows, occupancy grid, and rgb data.
    """
    try:
        # Start the simulation and get the client ID
        clientId = startSimulation()

        # Get the positions, weights, rows, and object handles from the simulation
        positions, weights, rows, object_handles = get_positions_objects.get_positions(clientId)

        # Generate the RGB data
        rgb = generate_rgb.generate_rgb(clientId, name_folder)

        # Generate the occupancy grid
        occupancy_grid = generate_cccupancy_grid.main(clientId, object_handles, name_folder, rgb)

        # Close the simulation
        close_simulation(clientId)

        # Print the occupancy grid
        print(occupancy_grid)

        # Return the data as a dictionary
        return {
            'weights': weights,
            'positions': positions,
            'rows': rows,
            'occupancy_grid': occupancy_grid,
            'rgb': rgb
        }
    except Exception as e:
        # Print any exceptions that occur
        print(e)


def main():
    """
    Main function to execute the steps of the application.
    """
    # Create a new folder
    name_folder = create_folder.create_folder()

    # Get the data and print it
    data = get_data(name_folder)
    print(data)

    # Return the data
    return data


# Execute main function
if __name__ == '__main__':
    main()
