def generate_forbidden_connections(rows):
    """
    Function to generate forbidden connections between nodes.

    Parameters:
    rows (dict): A dictionary containing the rows of nodes.

    Returns:
    list: A list of tuples representing the forbidden connections between nodes.
    """
    forbidden_connections = []
    num_rows = len(rows)

    # Iterate over each row except the first and last
    for h in range(1, num_rows-1):
        current_row = rows[f'h{h}']
        prev_row = rows[f'h{h - 1}'] if h > 1 else None
        next_row = rows[f'h{h + 1}'] if h < num_rows else None

        # Iterate over each node in the current row
        for obj_name, (obj_pos, obj_index) in current_row.items():
            # If the node is a 'c' node
            if 'c' in obj_name:
                # If there is a previous row, add a forbidden connection to each node in the previous row
                if prev_row:
                    for prev_obj_name, (prev_obj_pos, prev_obj_index) in prev_row.items():
                        forbidden_connections.append((obj_index, prev_obj_index))
                # If there is a next row, add a forbidden connection to each node in the next row
                if next_row:
                    for next_obj_name, (next_obj_pos, next_obj_index) in next_row.items():
                        forbidden_connections.append((obj_index, next_obj_index))

    return forbidden_connections