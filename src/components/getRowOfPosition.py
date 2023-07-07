import numpy as np

def find_row_for_position(rows, position):
    for row_name, row_data in rows.items():
        for object_name, (object_position, _) in row_data.items():
            if np.array_equal(object_position, position):
                return row_name
    return None