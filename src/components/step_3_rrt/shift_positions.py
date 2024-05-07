from src.steps import config


def shift_positions(ordered_positions):
    """
    Shift the positions to fit within the map size.

    Args:
        ordered_positions (list): List of ordered positions.

    Returns:
        list: List of shifted positions.
    """
    size = config.SIZE_MAP
    scale_value = size / config.SCALE_MAP
    shifted_positions = []
    for position in ordered_positions:
        shifted_x = (scale_value * position[0] + size / 2)
        shifted_y = (scale_value * position[1] + size / 2)
        shifted_positions.append((shifted_x, shifted_y))
    return shifted_positions
