import math

import numpy as np

from src.RRT.RRT_C import Node
from src.steps import config


def check_kinematic_constraints(node1, node2):
    dy = node2.row - node1.row
    dx = node2.col - node1.col
    angle = math.atan2(dy, dx)
    print(abs(angle) <= math.radians(95))

    return abs(angle) >= math.radians(95)


if __name__ == '__main__':

    node1 = Node(row=0, col=0)
    node2 = Node(row=1, col=1)
    max_angle = math.radians(45)  # Ángulo máximo permitido, convertido a radianes

    if check_kinematic_constraints(node1, node2):
        print("El ángulo cumple con la restricción.")
    else:
        print("El ángulo no cumple con la restricción.")
