import time
import math
from scipy.interpolate import interp1d

from src.components.step_4_controller.pid_controller import main_pid
from src.components.step_4_controller.speed_mapping import speed_mapping
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
    error_code_pionner, pioneer_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    print(error_code_pionner)
    error_code_left, left_motor_handle = sim.simxGetObjectHandle(clientID, 'leftMotor',
                                                                 sim.simx_opmode_blocking)
    print(error_code_left, left_motor_handle)
    error_code_right, right_motor_handle = sim.simxGetObjectHandle(clientID, 'rightMotor',
                                                                   sim.simx_opmode_blocking)
    print(error_code_right, right_motor_handle)
    for i in range(1, 30):
        # Get the position of the robot
        error_code_position, position = sim.simxGetObjectPosition(clientID, pioneer_handle, -1,
                                                                  sim.simx_opmode_blocking)
        print(error_code_position, position)
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 1, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.5, sim.simx_opmode_oneshot)
        time.sleep(0.1)

    close_simulation(clientID)


if __name__ == '__main__':
    # Uso de la función
    # interpolate_path()
    main_pid()
    move_controller()
