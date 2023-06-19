import matplotlib.pyplot as plt
import networkx as nx


# Costo computacional O(n) n-> len nodes
def print_solution(data, manager, routing, solution):
    """Prints solution on console and visualizes the graph."""
    print(f'Objective: {solution.ObjectiveValue()}')
    total_distance = 0
    total_load = 0

    # Create graph of the route
    G = nx.DiGraph()
    for i in range(manager.GetNumberOfNodes()):
        G.add_node(i, pos=(data['positions'][i][0], data['positions'][i][1]), weight=data['demands'][i])

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            next_node_index = manager.IndexToNode(solution.Value(routing.NextVar(index)))
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            G.add_edge(node_index, next_node_index)

        pos = nx.get_node_attributes(G, 'pos')
        node_labels = {i: '({}, {})'.format(i, data['demands'][i]) for i in G.nodes()}
        edge_labels = {(u, v): data['distance_matrix'][u][v] for u, v in G.edges() if (u, v) in data['distance_matrix']}
        plt.figure(figsize=(8, 8))
        nx.draw_networkx(G, pos, with_labels=False, node_size=1500, node_color='lightblue')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        nx.draw_networkx_labels(G, pos, labels=node_labels)
        plt.xlabel('X-axis (meters)')
        plt.ylabel('Y-axis (meters)')
        plt.title('Prioritized Harvest Collection Route')
        plt.show()
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index), route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)

# Costo computacional O(V*N) V-> len v; N-> len nodes
def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        routes.append(route)
    return routes