import time
import numpy as np
from scipy.interpolate import CubicSpline

from src.components.step1_get_data.close_simulation_coppelia import close_simulation
from src.components.step1_get_data.start_simulation_coppelia import startSimulation
from src.coppelia import sim


def get_gyro_gps_handles(clientID):
    _, gyro_handle = sim.simxGetObjectHandle(clientID, 'Gyro#0', sim.simx_opmode_blocking)
    _, gps_handle = sim.simxGetObjectHandle(clientID, 'GPS#0', sim.simx_opmode_blocking)
    return gyro_handle, gps_handle


def get_pionner_handles(clientID):
    _, pioneer_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    _, left_motor_handle = sim.simxGetObjectHandle(clientID, 'leftMotor', sim.simx_opmode_blocking)
    _, right_motor_handle = sim.simxGetObjectHandle(clientID, 'rightMotor', sim.simx_opmode_blocking)
    return left_motor_handle, right_motor_handle, pioneer_handle


def get_gps_data(clientID, gps_handle):
    _, gps_position = sim.simxGetObjectPosition(clientID, gps_handle, -1, sim.simx_opmode_blocking)
    # Aquí coloca tu lógica para obtener datos del sensor GPS en CoppeliaSim
    position_x = gps_position[0]
    position_y = gps_position[1]
    position_z = gps_position[2]
    return position_x, position_y, position_z


def get_gyro_data(clientID, gyro_handle):
    _, gyro_orientation = sim.simxGetObjectOrientation(clientID, gyro_handle, -1, sim.simx_opmode_blocking)
    return gyro_orientation[2]


def get_pionner_poisiton(clientID, pionner_handle):
    error_code_position, position = sim.simxGetObjectPosition(clientID, pionner_handle, -1,
                                                              sim.simx_opmode_blocking)
    position_x = position[0]
    position_y = position[1]
    position_z = position[2]
    print(error_code_position, position)
    return position_x, position_y, position_z


def move_robot(clientID, left_motor_handle, right_motor_handle, v_left, v_right):
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, v_left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, v_right, sim.simx_opmode_oneshot)


def generate_trajectory(x, y):
    # Interpolación cúbica para generar una trayectoria suave
    spline_x = CubicSpline(x, y)
    smooth_x = np.linspace(min(x), max(x), 100)
    smooth_y = spline_x(smooth_x)
    return smooth_x, smooth_y


def pid_controller(clientID, left_motor_handle, right_motor_handle, pionner_handle, path_x, path_y, ):
    # Parámetros del PID
    kp = 1.0
    ki = 0.1
    kd = 0.01

    # Inicialización de errores acumulativos e instantáneos
    integral = 0
    prev_error = 0

    # Obtener la trayectoria suave
    smooth_x, smooth_y = generate_trajectory(path_x, path_y)
    # left_motor_handle, right_motor_handle, pioneer_handle = get_pionner_handles(clientID)

    # Loop principal del controlador
    for i in range(len(smooth_x)):
        position_x, position_y, position_z = get_pionner_poisiton(clientID, pionner_handle)
        # gyro_data = get_gyro_data(clientID, gyro_handle)

        # Calcular error de posición en la trayectoria
        error = np.sqrt((position_x - smooth_x[i]) ** 2 + (position_y - smooth_y[i]) ** 2)

        # Calcular términos PID
        proportional = kp * error
        integral += ki * error
        derivative = kd * (error - prev_error)

        # Calcular velocidad de referencia para ambas ruedas
        v_ref = proportional + integral + derivative

        # Aplicar control PID para ajustar velocidades de las ruedas
        v_left = v_ref
        v_right = v_ref

        # Mover el robot con las velocidades calculadas
        move_robot(clientID, left_motor_handle, right_motor_handle, v_left, v_right)

        # Actualizar error previo para el próximo ciclo
        prev_error = error

        # Esperar un breve momento antes de la siguiente iteración
        time.sleep(0.1)


def main_pid():
    clientID = startSimulation()
    left_motor_handle, right_motor_handle, pioneer_handle = get_pionner_handles(clientID)

    path_x = [0, 2]
    path_y = [0, 3]

    # Ejecutar el controlador PID con la trayectoria especificada
    pid_controller(clientID, left_motor_handle, right_motor_handle, pioneer_handle, path_x, path_y)

    # Finalizar la simulación
    close_simulation(clientID)


if __name__ == '__main__':
    main_pid()
