import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import traceback
from colorama import init, Fore

init()


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path

    def check_collision(self, p1, p2):

        delta = 1  # spacing between each sample point
        x1, y1 = p1
        x2, y2 = p2
        theta = np.arctan2(y2 - y1, x2 - x1)  # slope of the line
        sx = x1
        sy = y1
        for i in range(1, round(self.dis(p1, p2)) + 1):
            sx = round(x1 + i * delta * np.cos(theta))
            sy = round(y1 + i * delta * np.sin(theta))
            if not self.map_array[sx][sy]:
                return True
        return False

    def dis(self, point1, point2):

        return np.linalg.norm(np.array(point1) - np.array(point2))

    def uniform_sample(self, n_pts):

        self.graph.clear()

        row = np.linspace(0, self.size_row - 1, num=int(np.sqrt(n_pts)), dtype='int32')
        col = np.linspace(0, self.size_col - 1, num=int(np.sqrt(n_pts)), dtype='int32')
        for r in row:
            for c in col:
                p = (r, c)
                if self.map_array[p[0], p[1]]:
                    self.samples.append(p)

    def random_sample(self, n_pts):

        self.graph.clear()

        for i in range(n_pts):
            p = np.random.randint([self.size_row - 1, self.size_col - 1])
            if self.map_array[p[0], p[1]]:
                self.samples.append(tuple(p))

    def draw_map(self, paths, graphs, goals, sampling_method):
        fig, ax = plt.subplots()
        ax.imshow(self.map_array, cmap='gray', origin='lower')

        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g', label='start')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r', label='goal')

        for index, path in enumerate(paths):
            graph = graphs[index]

            if path:
                # Add temporary start and goal edge to the path
                final_path_edge = list(zip(path[:-1], path[1:]))

                # Draw consecutive edges between path nodes
                for i in range(len(final_path_edge)):
                    edge = final_path_edge[i]
                    # if edge[0] != 'start' and edge[0] != 'goal' and edge[1] != 'start' and edge[1] != 'goal':
                    nx.draw_networkx_edges(graph, pos=pos, edgelist=[edge], width=2, edge_color='b', label='path')

                    # Draw path nodes
                nx.draw_networkx_nodes(graph, pos=pos, nodelist=path[1:-1], node_size=8, node_color='b')
                # Show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.title(sampling_method)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):

        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)

        pairs = []
        self.kdtree = spatial.KDTree(self.samples)
        _, pairs_idx = self.kdtree.query(self.samples, k=[1, 2, 3, 4, 5, 6, 7, 8])
        for i in range(len(pairs_idx)):
            for j in range(1, len(pairs_idx[0])):
                p1 = self.samples[i]
                p2 = self.samples[pairs_idx[i][j]]
                if not self.check_collision(p1, p2):
                    pairs.append((pairs_idx[i][0], pairs_idx[i][j], self.dis(p1, p2)))

        self.graph.add_nodes_from(range(1, len(self.samples) + 1))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))

    def search(self, goals, sampling_method):
        try:
            paths = []
            graphs = []
            for index, (position) in enumerate(goals):
                if index == 2:
                    break
                start = position
                goal = goals[index + 1]
                self.path = []
                start = [start[1], start[0]]
                goal = [goal[1], goal[0]]
                self.samples.append(start)
                self.samples.append(goal)
                self.graph.add_nodes_from(['start', 'goal'])
                start_pairs = []
                goal_pairs = []
                _, start_pairs_idx = self.kdtree.query(start, k=[1, 2, 3, 4, 5])
                _, goal_pairs_idx = self.kdtree.query(goal, k=[1, 2, 3, 4, 5])
                for s_idx in start_pairs_idx:
                    p = self.samples[s_idx]
                    if not self.check_collision(start, p):
                        start_pairs.append(("start", s_idx, self.dis(start, p)))
                for g_idx in goal_pairs_idx:
                    p = self.samples[g_idx]
                    if not self.check_collision(goal, p):
                        goal_pairs.append(("goal", g_idx, self.dis(goal, p)))
                self.graph.add_weighted_edges_from(start_pairs)
                self.graph.add_weighted_edges_from(goal_pairs)

                try:
                    self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
                    path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start',
                                                                                             'goal')
                    print("The path length is %.2f" % path_length)
                except nx.exception.NetworkXNoPath:
                    print("No path found")
                paths.append(self.path)
                graphs.append(self.graph)
                if index == 1:
                    self.draw_map(paths, graphs, goals, sampling_method)
                self.samples.pop(-1)
                self.samples.pop(-1)
                self.graph.remove_nodes_from(['start', 'goal'])
                self.graph.remove_edges_from(start_pairs)
                self.graph.remove_edges_from(goal_pairs)
                # if index > 0:
                #     self.samples.pop(-1)
                #     self.samples.pop(-1)
                #     self.graph.remove_nodes_from(['start', 'goal'])
                #     self.graph.remove_edges_from(start_pairs)
                #     self.graph.remove_edges_from(goal_pairs)



        except Exception as e:
            print(Fore.RED + e)
            traceback.print_exc()
