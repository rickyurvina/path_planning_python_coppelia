
def get_ordered_positions(route, positions):
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])

    return ordered_positions
