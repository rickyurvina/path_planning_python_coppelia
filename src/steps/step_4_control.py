import time
import math
from scipy.interpolate import interp1d
from src.components.move_controller.speed_mapping import speed_mapping
from src.components.step1_get_data.close_simulation_coppelia import close_simulation
from src.coppelia import sim
from src.components.step1_get_data.start_simulation_coppelia import startSimulation


def interpolate_path(path=[(0, 0), (2, 3), (5, 1), (8, 4), (10, 0)]):
    # Assign time values to each point in the path
    rrt_path_with_time = [(x, y, time) for time, (x, y) in enumerate(path)]

    print(rrt_path_with_time)

    # Create interpolation functions
    x_values, y_values, time_values = zip(*rrt_path_with_time)
    interpolate_x = interp1d(time_values, x_values, kind='linear', fill_value='extrapolate')
    interpolate_y = interp1d(time_values, y_values, kind='linear', fill_value='extrapolate')

    return interpolate_x, interpolate_y


def move_controller():
    # Connect to CoppeliaSim
    clientID = startSimulation()

    if clientID != -1:
        print("Successful connection to CoppeliaSim")
    else:
        print("Error connecting to CoppeliaSim")
        exit()
    # Get the handle of the Pioneer 3DX robot
    _, pioneer_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    _, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 1, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 1, sim.simx_opmode_oneshot)
    # sim.simxSetObjectPosition(clientID, pioneer_handle, -1, [0, 0, 0], sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot)
    close_simulation(clientID)


if __name__ == '__main__':
    # Uso de la función
    interpolate_path()
