from scipy import spatial


def get_nearest_node(self, point):
    """
    Get the nearest node to a given point among the vertices.

    Args:
        self: The RRT object.
        point (tuple): The coordinates of the point.

    Returns:
        Node: The nearest node to the given point.
    """
    samples = [[v.row, v.col] for v in self.vertices]
    kdtree = spatial.cKDTree(samples)
    coord, ind = kdtree.query(point)
    return self.vertices[ind]
