from src.coppelia import sim


def startSimulation():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    try:
        if clientID == -1:
            print('No se pudo conectar con CoppeliaSim')
            exit()
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
        return clientID
    except Exception as e:
        print(e)
        if clientID != -1:
            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
            sim.simxFinish(clientID)

    return clientID

def main():
    startSimulation()


if __name__ == '__main__':
    main()
