from src.coppelia import sim


def startSimulation():
    """
    Function to start a simulation in CoppeliaSim.

    Returns:
    clientID (int): The ID of the client for which the simulation is started.
    """
    # Finish any previous connections
    sim.simxFinish(-1)
    # Start a new connection
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    try:
        # Check if the connection was successful
        if clientID == -1:
            print('No se pudo conectar con CoppeliaSim')
            exit()
        # Start the simulation
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
        return clientID
    except Exception as e:
        print(e)
        # If an error occurred and the clientID is valid, stop the simulation and finish the connection
        if clientID != -1:
            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
            sim.simxFinish(clientID)

    return clientID


def main():
    """
    Main function to start the simulation.
    """
    startSimulation()


if __name__ == '__main__':
    # Call the main function
    main()
