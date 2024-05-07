from scipy import spatial


def get_neighbors(self, new_node, neighbor_size):
    """
    Get the neighbors of a new node within a specified neighbor size.

    Args:
        self: The RRT object.
        new_node (Node): The new node.
        neighbor_size (float): The maximum distance to search for neighbors.

    Returns:
        list: A list of neighboring nodes within the specified neighbor size.
    """
    samples = [[v.row, v.col] for v in self.vertices]
    kdtree = spatial.cKDTree(samples)
    ind = kdtree.query_ball_point([new_node.row, new_node.col], neighbor_size)
    neighbors = [self.vertices[i] for i in ind]
    neighbors.remove(new_node)
    return neighbors
