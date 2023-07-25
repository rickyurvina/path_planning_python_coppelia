def get_ordered_weights(route, weights):
    get_ordered_weights = []
    for row in range(len(route)):
        get_ordered_weights.append(weights[route[row]])

    return get_ordered_weights
