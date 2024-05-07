import numpy as np

def find_row_for_position(rows, position):
    """
    Function to find the row for a given position.

    Parameters:
    rows (dict): A dictionary containing the rows of nodes.
    position (list): The position of the node.

    Returns:
    str: The name of the row where the node is located. If the node is not found, returns None.
    """
    # Iterate over each row
    for row_name, row_data in rows.items():
        # Iterate over each node in the row
        for object_name, (object_position, _) in row_data.items():
            # If the position of the node matches the given position
            if np.array_equal(object_position, position):
                # Return the name of the row
                return row_name
    # If the node is not found, return None
    return None