from src.components.common import load_files  # Importing load_files function from common module
from src.components.step_2_tsp.create_data_tsp import \
    create_data_model  # Importing create_data_model function from step_2_tsp module


def get_routes(solution, routing, manager):
    """
    Get vehicle routes from a solution and store them in an array.

    Args:
        solution: Solution object containing the computed solution to the vehicle routing problem.
        routing: Routing index manager.
        manager: Manager object containing node indices.

    Returns:
        List: A list of routes, where each route is represented as a list of node indices.
    """
    # Get vehicle routes and store them in a two-dimensional array whose
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


# TODO REFACTOR THIS MAIN FOR THIS FILE
if __name__ == '__main__':
    # Load solution data from files
    data = load_files.load_solution_data()

    # Create data model
    data_model = create_data_model(data['positions'], data['weights'], data['rows'])

    # Get routes using the loaded solution, routing, and manager
    route = get_routes(data['solution'], data['routing'], data['manager'])

    # Print the routes
    print(route)

    # Print a warning message indicating this script should not be run directly
    print('This script is not meant to be run directly. Run main.py instead.')
