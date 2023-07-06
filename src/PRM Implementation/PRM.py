# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###

        ### Approach 2 ###
        # min_dis = 1
        # p1 = np.array(p1)
        # p2 = np.array(p2)
        # mid_pt = np.round(1/2*(p1+p2)).astype('int')
        # while self.dis(p1,mid_pt) >=min_dis:
        #     if not self.map_array[mid_pt[0]][mid_pt[1]]:
        #         return True
        #     if self.dis(p1,mid_pt)==min_dis or self.dis(p1,mid_pt)==np.sqrt(2):
        #         break
        #     mid_pt = np.round(1/2*(p1+mid_pt)).astype('int')
            
        # mid_pt = np.round(1/2*(p1+p2)).astype('int')
        # while self.dis(p2,mid_pt)>=min_dis: 
        #     if not self.map_array[mid_pt[0]][mid_pt[1]]:
        #         return True
        #     if self.dis(p2,mid_pt)==min_dis or self.dis(p2,mid_pt)==np.sqrt(2):
        #         break
        #     mid_pt = np.round(1/2*(mid_pt+p2)).astype('int')
            
        # return False

        delta = 1 # spacing between each sample point
        x1,y1 = p1
        x2,y2 = p2
        theta = np.arctan2(y2-y1,x2-x1) # slope of the line
        sx=x1
        sy=y1
        for i in range(1,round(self.dis(p1,p2))+1):
            sx = round(x1+i*delta*np.cos(theta))
            sy = round(y1+i*delta*np.sin(theta))
            if not self.map_array[sx][sy]:
                return True
        return False


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        return np.linalg.norm(np.array(point1)-np.array(point2))


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        row = np.linspace(0,self.size_row-1,num=int(np.sqrt(n_pts)),dtype='int32')
        col = np.linspace(0,self.size_col-1,num=int(np.sqrt(n_pts)),dtype='int32')
        for r in row:
            for c in col:
                p = (r,c)
                if self.map_array[p[0],p[1]]:
                    self.samples.append(p)

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        for i in range(n_pts):
            p = np.random.randint([self.size_row-1,self.size_col-1])
            if self.map_array[p[0],p[1]]:
                self.samples.append(tuple(p))


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        std_dev = 10
        for i in range(n_pts):
            p1 = np.random.uniform([self.size_row-1,self.size_col-1]).astype('int32')
            p2 = np.random.normal(p1,std_dev).astype('int32')
            if (0<=p2[0]<self.size_row) & (0<=p2[1]<self.size_col):
                if (self.map_array[p1[0],p1[1]]^self.map_array[p2[0],p2[1]]):
                    if self.map_array[p1[0],p1[1]]:
                        self.samples.append(tuple(p1))
                    else:
                        self.samples.append(tuple(p2))


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        std_dev = 20
        for i in range(n_pts):
            p1 = np.random.uniform([self.size_row-1,self.size_col-1]).astype('int32')           
            if not self.map_array[p1[0],p1[1]]:
                p2 = np.random.normal(p1,std_dev).astype('int32')
                if (0<=p2[0]<self.size_row) & (0<=p2[1]<self.size_col) and not self.map_array[p2[0],p2[1]]:
                    mid_pt = tuple(np.mean([p1,p2],axis=0,dtype='int32'))
                    if self.map_array[mid_pt[0],mid_pt[1]]:
                        self.samples.append(mid_pt)
                    


    def draw_map(self, sampling_method):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.title(sampling_method)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []
        self.kdtree = spatial.KDTree(self.samples)
        _,pairs_idx = self.kdtree.query(self.samples,k=[1,2,3,4,5,6,7,8])
        for i in range(len(pairs_idx)):
            for j in range(1,len(pairs_idx[0])):
                p1 = self.samples[i]
                p2 = self.samples[pairs_idx[i][j]]
                if not self.check_collision(p1,p2):
                    pairs.append((pairs_idx[i][0],pairs_idx[i][j],self.dis(p1,p2)))
        
        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(range(1,len(self.samples)+1))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal, sampling_method):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []
        _,start_pairs_idx = self.kdtree.query(start,k=[1,2,3,4,5])
        _,goal_pairs_idx = self.kdtree.query(goal,k=[1,2,3,4,5])
        for s_idx in start_pairs_idx:
            p = self.samples[s_idx]
            if not self.check_collision(start,p):
                start_pairs.append(("start",s_idx,self.dis(start,p)))
        for g_idx in goal_pairs_idx:
            p = self.samples[g_idx]
            if not self.check_collision(goal,p):
                goal_pairs.append(("goal",g_idx,self.dis(goal,p)))
        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map(sampling_method)

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        