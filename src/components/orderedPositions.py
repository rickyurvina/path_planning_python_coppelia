from src.components import loadFiles


def get_ordered_positions(route, positions):
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])

    return ordered_positions


if __name__ == '__main__':
    data = loadFiles.load_solution_data()
    print(get_ordered_positions(data['route'], data['positions']))
