from src.components.step_3_rrt.path_to_file import export_to_excel_and_json


def get_path_coordinates(self):
    """
    Get the x and y coordinates of the path.

    Args:
        self: Instance of the RRT class.

    Returns:
        tuple: Tuple containing lists of x and y coordinates.
    """
    # Initialize lists to store x and y coordinates
    x_positions = []
    y_positions = []

    # Traverse the node tree and store the coordinates
    if self.name_method == "IRRT":
        print("The method is IRRT")
        x_positions = [point[0] for point in self.path]
        y_positions = [point[1] for point in self.path]
    else:
        current_node = self.path[0]
        while current_node is not None:
            x_positions.append(current_node.col)
            y_positions.append(current_node.row)
            current_node = current_node.parent
        x_positions.reverse()
        y_positions.reverse()

    x_y_positions = list(zip(x_positions, y_positions))
    export_to_excel_and_json(x_y_positions, self.name_folder)

    return x_positions, y_positions
