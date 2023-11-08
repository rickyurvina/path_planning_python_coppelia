from src.components.common import load_files


def get_ordered_positions(route, positions):
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])

    return ordered_positions


if __name__ == '__main__':
    data = load_files.load_solution_data()
    print(get_ordered_positions(data['route'], data['positions']))
