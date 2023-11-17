import numpy as np


class UnicycleRobot:
    def __init__(self, wheel_radius, L, dt, accel_max, max_angle_curvature):
        self.wheel_radius = wheel_radius
        self.L = L
        self.dt = dt
        self.accel_max = accel_max
        # TODO MAX ANGLE CURVATURE
        self.max_angle_curvature = max_angle_curvature

    def kinematics(self, w_r, w_l):
        v = self.wheel_radius * (w_r + w_l) / 2
        omega = self.wheel_radius * (w_r - w_l) / self.L
        return v, omega

    def steer(self, state, velocity, angle_curvature):
        x, y, theta, v = state

        # Aplicar restricciones cinemáticas
        v = np.clip(velocity, -self.accel_max, self.accel_max)
        angle_curvature = np.clip(angle_curvature, -self.max_angle_curvature, self.max_angle_curvature)

        # Actualizar posición y orientación
        x += v * np.cos(theta) * self.dt
        y += v * np.sin(theta) * self.dt
        theta += (v / self.L) * np.tan(angle_curvature) * self.dt

        return np.array([x, y, theta, v])


if __name__ == '__main__':
    # Parámetros del robot
    wheel_radius = 0.1  # Radio de las ruedas
    L = 0.5  # Longitud entre ejes
    dt = 0.1  # Paso de tiempo
    accel_max = 1.0  # Aceleración máxima
    max_angle_curvature = 0.5  # Ángulo de giro máximo

    # Crear una instancia del robot
    robot = UnicycleRobot(wheel_radius, L, dt, accel_max, max_angle_curvature)

    # Estado inicial del robot [x, y, theta, v]
    initial_state = np.array([0.0, 0.0, 0.0, 0.0])

    for _ in range(10):
        # Genera valores aleatorios para la velocidad y el ángulo de giro
        velocity = np.random.uniform(0, accel_max)  # Velocidad aleatoria
        angle_curvature = np.random.uniform(-max_angle_curvature, max_angle_curvature)  # Ángulo de giro aleatorio

        # Aplica el modelo cinemático del robot para calcular el nuevo estado
        initial_state = robot.steer(initial_state, velocity, angle_curvature)

        # Imprime el nuevo estado del robot
        print("Estado del robot:", initial_state)
