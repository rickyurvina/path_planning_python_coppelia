def get_total_loaded(data, manager, routing, solution):
    """
    Function to calculate the total load of a route.

    Parameters:
    data (dict): The data for the problem.
    manager (RoutingIndexManager): Index manager for the routing problem.
    routing (RoutingModel): The routing problem.
    solution (Assignment): The solution to the routing problem.

    Returns:
    int: The total load of the route.
    """
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            index = solution.Value(routing.NextVar(index))

    return route_load


def get_total_length(data, routing, solution):
    """
    Function to calculate the total length of a route.

    Parameters:
    data (dict): The data for the problem.
    routing (RoutingModel): The routing problem.
    solution (Assignment): The solution to the routing problem.

    Returns:
    int: The total length of the route.
    """
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

    return route_distance