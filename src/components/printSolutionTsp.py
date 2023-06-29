import matplotlib.pyplot as plt
import networkx as nx
from src.components import saveFiles


def print_solution_tsp(data, manager, routing, solution, name_folder):
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
        plt.xlabel('x-coordenadas (m)')
        plt.ylabel('y-coordenadas (m)')
        plt.title('Ruta Priorizada de Recolección de Cosecha')
        plt.savefig(saveFiles.get_name_to_save_plot(name_folder, 'solution_tps'), dpi=500)
        plt.show()
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index), route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
