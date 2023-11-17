from scipy import spatial


def get_nearest_node(self, point):
    samples = [[v.row, v.col] for v in self.vertices]
    kdtree = spatial.cKDTree(samples)
    coord, ind = kdtree.query(point)
    return self.vertices[ind]
