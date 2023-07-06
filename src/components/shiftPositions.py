def shift_positions(ordered_positions):
    scale_value = 1000 / 15
    shifted_positions = []
    for position in ordered_positions:
        shifted_x = (scale_value * position[0] + 500)
        shifted_y = (scale_value * position[1] + 500)
        shifted_positions.append((shifted_x, shifted_y))
    return shifted_positions