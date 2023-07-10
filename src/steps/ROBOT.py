class SkidSteerRobot:
    def __init__(self, wheel_distance):
        self.wheel_distance = wheel_distance

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        # Calcula las velocidades de las ruedas izquierda y derecha
        wheel_left_velocity = (2 * linear_velocity - angular_velocity * self.wheel_distance) / (2 * self.wheel_distance)
        wheel_right_velocity = (2 * linear_velocity + angular_velocity * self.wheel_distance) / (2 * self.wheel_distance)
        return wheel_left_velocity, wheel_right_velocity
