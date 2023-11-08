from src.coppelia import sim


def close_simulation(clientID):
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)


def main():
    close_simulation(0)


if __name__ == '__main__':
    main()
