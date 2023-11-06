from src.components import load_files
from src.components.create_data_tsp import create_data_model


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


# TODO REFACTOR THIS MAIN FOR THIS FILE
if __name__ == '__main__':
    data = load_files.load_solution_data()
    data_model = create_data_model(data['positions'], data['weights'], data['rows'])
    route = get_routes(data['solution'], data['routing'], data['manager'])
    print(route)
    print('This script is not meant to be run directly. Run main.py instead.')
