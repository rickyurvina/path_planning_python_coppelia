class UnicycleRobot:
    def __init__(self, wheel_radius, distance_to_body):
        self.wheel_radius = wheel_radius
        self.distance_to_body = distance_to_body

    def kinematics(self, w_r, w_l):
        v = self.wheel_radius * (w_r + w_l) / 2
        omega = self.wheel_radius * (w_r - w_l) / self.distance_to_body
        return v, omega

# Ejemplo de uso
if __name__ == "__main__":
    robot = UnicycleRobot(wheel_radius=0.1, distance_to_body=0.5)
    w_r = 2.0  # Velocidad angular de la rueda derecha
    w_l = 1.5  # Velocidad angular de la rueda izquierda
    v, omega = robot.kinematics(w_r, w_l)
    print("Velocidad lineal:", v)
    print("Velocidad angular:", omega)
