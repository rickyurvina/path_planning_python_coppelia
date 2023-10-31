from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from src.components.printSolutionTsp import print_solution_tsp
from src.components.createDataTsp import create_data_model
from src.components.displaySolution import plots_positions
from src.components.routesOfSolution import get_routes
from src.components.orderedPositions import get_ordered_positions
from src.components import getTotalLoaded, createFolder
from src.steps import getDataCoppelia


def main(positions, weights, rows, name_folder):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(positions, weights, rows)

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

    penalty = 1000
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Add Forbidden Connections constraint.
    for connection in data['forbidden_connections']:
        routing.solver().Add(routing.NextVar(manager.NodeToIndex(connection[0])) !=
                             manager.NodeToIndex(connection[1]))  # define las restricciones en las variables.
        routing.solver().Add(routing.NextVar(manager.NodeToIndex(connection[1])) !=
                             manager.NodeToIndex(connection[0]))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # LOCAL_CHEAPEST_INSERTION || CHRISTOFIDES || SAVINGS || PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)  # SIMULATED_ANNEALING || GUIDED_LOCAL_SEARCH || TABU_SEARCH

    search_parameters.time_limit.FromSeconds(1)
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution_tsp(data, manager, routing, solution, name_folder)
        plots_positions(get_routes(solution, routing, manager)[0], data, rows, name_folder)
        route = get_routes(solution, routing, manager)[0]
        ordered_positions = get_ordered_positions(route, data['positions'])
        total_loaded = getTotalLoaded.get_total_loaded(data, manager, routing, solution)
        total_length = getTotalLoaded.get_total_length(data, routing, solution)
        return ordered_positions, route, total_loaded, total_length
    else:
        print("Solver status: Error al resolver el problema")


if __name__ == '__main__':
    name_folder = createFolder.create_folder()
    data = getDataCoppelia.get_data(name_folder)
    main(data['positions'], data['weights'], data['rows'], name_folder)
