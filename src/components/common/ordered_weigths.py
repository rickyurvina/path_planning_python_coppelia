def get_ordered_weights(route, weights):
    """
    Function to get ordered weights based on a given route.

    Parameters:
    route (list): A list of indices representing the route.
    weights (list): A list of weights.

    Returns:
    list: A list of weights ordered according to the route.
    """
    # Initialize an empty list to store the ordered weights
    ordered_weights = []

    # Iterate over the route
    for row in range(len(route)):
        # Append the weight corresponding to the current index in the route to the ordered_weights list
        ordered_weights.append(weights[route[row]])

    # Return the list of ordered weights
    return ordered_weights
