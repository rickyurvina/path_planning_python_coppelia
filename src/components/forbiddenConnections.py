def generate_forbidden_connections(rows):
    forbidden_connections = []
    num_rows = len(rows)

    for h in range(1, num_rows + 1):
        current_row = rows[f'h{h}']
        prev_row = rows[f'h{h - 1}'] if h > 1 else None
        next_row = rows[f'h{h + 1}'] if h < num_rows else None

        for obj_name, (obj_pos, obj_index) in current_row.items():
            if 'c' in obj_name:
                if prev_row:
                    for prev_obj_name, (prev_obj_pos, prev_obj_index) in prev_row.items():
                        forbidden_connections.append((obj_index, prev_obj_index))
                if next_row:
                    for next_obj_name, (next_obj_pos, next_obj_index) in next_row.items():
                        forbidden_connections.append((obj_index, next_obj_index))

    return forbidden_connections
