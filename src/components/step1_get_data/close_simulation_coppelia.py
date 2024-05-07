from src.coppelia import sim
from src.steps import config


def close_simulation(clientID):
    """
    Function to close the simulation in CoppeliaSim.

    Parameters:
    clientID (int): The ID of the client for which the simulation will be closed.

    Returns:
    int: Returns 0 after successfully closing the simulation.
    """
    # Check if the simulation is online
    if config.ON_LINE:
        # Stop the simulation for the specified client
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
        # Finish the connection to the client
        sim.simxFinish(clientID)
    return 0


def main():
    """
    Main function to close the simulation if it is online.
    """
    # Check if the simulation is online
    if config.ON_LINE:
        # Close the simulation for client 0
        close_simulation(0)


if __name__ == '__main__':
    # Call the main function
    main()
