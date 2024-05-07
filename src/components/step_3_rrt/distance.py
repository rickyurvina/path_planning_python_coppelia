import numpy as np


def distance(node1, node2):
    """
    Calculate the Euclidean distance between two nodes.

    Args:
        node1 (Node): First node.
        node2 (Node): Second node.

    Returns:
        float: Euclidean distance between the two nodes.
    """
    return np.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
