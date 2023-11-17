import numpy as np


def dis(self, node1, node2):
    return np.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
