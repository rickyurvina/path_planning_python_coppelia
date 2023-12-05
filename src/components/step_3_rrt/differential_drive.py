import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class DifferentialDriveRobot:
    def __init__(self, wheelbase, wheel_radius):
        self.wheelbase = wheelbase
        self.wheel_radius = wheel_radius
        self.pose = np.array([0.0, 0.0, 0.0])

    def update_pose(self, v_left, v_right, dt):
        v_left = float(v_left)
        v_right = float(v_right)

        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheelbase

        self.pose[0] += v * np.cos(self.pose[2]) * dt
        self.pose[1] += v * np.sin(self.pose[2]) * dt
        self.pose[2] += omega * dt


if __name__ == "__main__":
    wheelbase = 1.0
    wheel_radius = 0.2
    robot = DifferentialDriveRobot(wheelbase, wheel_radius)

    for _ in range(100):
        robot.update_pose(1.0, 1.0, 0.1)
        print(robot.pose)
