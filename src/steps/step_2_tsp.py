# Import necessary modules and configurations
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from src.components.step_2_tsp.create_data_tsp import create_data_model
from src.components.step_2_tsp.display_solution import plots_positions
from src.components.step_2_tsp.routes_of_solution import get_routes
from src.components.common.ordered_positions import get_ordered_positions
from src.components.step_2_tsp import get_total_loaded
from src.components.common import load_files
from src.components import create_folder
from src.steps import config
from src.steps.step_1_get_data import get_data
import random


def main(positions, weights, rows, name_folder):
    """
    Main function to solve the CVRP problem.
    Args:
        positions (list): The positions of the nodes.
        weights (list): The weights of the nodes.
        rows (int): The number of rows.
        name_folder (str): The name of the folder where the data will be stored.
    Returns:
        tuple: A tuple containing the ordered positions, route, total loaded, and total length.
    """
    # Instantiate the data problem.
    data = create_data_model(positions, weights, rows)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['starts'], data["ends"])

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
        # print_solution_tsp(data, manager, routing, solution, name_folder)
        plt = plots_positions(get_routes(solution, routing, manager)[0], data, rows, name_folder)
        route = get_routes(solution, routing, manager)[0]
        ordered_positions = get_ordered_positions(route, data['positions'])
        total_loaded = get_total_loaded.get_total_loaded(data, manager, routing, solution)
        total_length = get_total_loaded.get_total_length(data, routing, solution)
        return ordered_positions, route, total_loaded, total_length
    else:
        print("Solver status: Error al resolver el problema")


def run_tsp_with_varied_capacity(data, name_folder, num_tests=1):
    """
    Function to run the TSP with varied capacity.
    Args:
        data (dict): The data dictionary.
        name_folder (str): The name of the folder where the data will be stored.
        num_tests (int, optional): The number of tests to run. Defaults to 1.
    """
    for i in range(num_tests):
        random_capacity = random.randint(8, 45)
        start_point_tsp = random.randint(0, len(data['positions']) - 1)
        # config.VEHICLE_CAPACITIES = 45
        config.START_POINT_TSP = 2
        main(data['positions'], data['weights'], data['rows'], name_folder)


if __name__ == '__main__':
    # Create a new folder
    name_folder = create_folder.create_folder()

    # If the application is set to online mode, get data
    if config.ON_LINE:
        data = get_data(name_folder)
    else:
        # If the application is set to offline mode, load solution data
        data = load_files.load_solution_data("solutions")

    # Run the TSP with varied capacity
    run_tsp_with_varied_capacity(data, name_folder)
