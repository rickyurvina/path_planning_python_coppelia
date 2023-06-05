from src.components import displayGraphW, getDynamicObjectsCoppelia, displayGraph
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import os
import pickle
import matplotlib.pyplot as plt
import networkx as nx

num_objects = 3
distance_matrix, weights, positions = getDynamicObjectsCoppelia.getData(num_objects)


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['positions'] = positions
    data['demands'] = weights
    data['weights'] = weights
    data['vehicle_capacities'] = [1500]
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    total_distance = 0
    total_load = 0

    # Crear gráfico del grafo de rutas
    G = nx.DiGraph()
    for i in range(manager.GetNumberOfNodes()):
        G.add_node(i, pos=(data['positions'][i][0], data['positions'][i][1]), weight=data['weights'][i])

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
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)

            next_node_index = manager.IndexToNode(solution.Value(routing.NextVar(index)))
            index = solution.Value(routing.NextVar(index))
            G.add_edge(node_index, next_node_index)

        pos = nx.get_node_attributes(G, 'pos')
        node_labels = nx.get_node_attributes(G, 'weight')
        edge_labels = {(u, v): data['distance_matrix'][u][v] for u, v in G.edges() if (u, v) in data['distance_matrix']}
        plt.figure(figsize=(8, 8))
        nx.draw_networkx(G, pos, with_labels=False, node_size=500, node_color='lightblue')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        nx.draw_networkx_labels(G, pos, labels=node_labels)
        plt.xlabel('Eje X (metros)')
        plt.ylabel('Eje Y (metros)')
        plt.title('Ruta priorizada de recolección de cosecha')
        plt.show()
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load

    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(100)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)

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

    routes = get_routes(solution, routing, manager)
    # Display the routes.
    route = routes[0]
    # displayGraphW.plotsPositions(route, data['demands'], data['positions'])
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])

    def save_workspace():
        # Crear un diccionario con todas las variables del programa
        variables = {
            'route': route,
            'ordered_positions': ordered_positions,
        }
        # Guardar las variables en un archivo pickle
        save_file(variables)

    def save_file(variables):
        folder = "../files/routes"
        prefix = "routes"
        ext = ".pickle"

        # Verificar la existencia de archivo y establecer el contador
        i = 1
        while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
            i += 1

        # Guardar el workspace con el nombre adecuado
        filename = os.path.join(folder, prefix + str(i) + ext)

        with open(filename, 'wb') as f:
            pickle.dump(variables, f)
        print('Variables guardadas en el archivo "routes.pickle"')

    save_workspace()


if __name__ == '__main__':
    main()
