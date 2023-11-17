from scipy import spatial


def get_neighbors(self, new_node, neighbor_size):
    samples = [[v.row, v.col] for v in self.vertices]
    kdtree = spatial.cKDTree(samples)
    ind = kdtree.query_ball_point([new_node.row, new_node.col], neighbor_size)
    neighbors = [self.vertices[i] for i in ind]
    neighbors.remove(new_node)
    return neighbors
