from src.components.common import load_files


def get_ordered_positions(route, positions):
    """
    Function to get ordered positions based on a given route.

    Parameters:
    route (list): A list of indices representing the route.
    positions (list): A list of positions.

    Returns:
    list: A list of positions ordered according to the route.
    """
    # Initialize an empty list to store the ordered positions
    ordered_positions = []

    # Iterate over the route
    for row in range(len(route)):
        # Append the position corresponding to the current index in the route to the ordered_positions list
        ordered_positions.append(positions[route[row]])

    # Return the list of ordered positions
    return ordered_positions


if __name__ == '__main__':
    # Load solution data
    data = load_files.load_solution_data()

    # Print the ordered positions based on the 'route' and 'positions' in the loaded data
    print(get_ordered_positions(data['route'], data['positions']))
