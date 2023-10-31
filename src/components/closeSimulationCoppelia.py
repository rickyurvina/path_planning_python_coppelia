from src.coppelia import sim


def closeSimulation(clientID):
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)

def main():
    closeSimulation(0)

if __name__ == '__main__':
    main()