import traceback
import numpy as np
from colorama import Fore

from src.components.step_3_rrt.smooth_path import smooth_path


def to_smooth_path(self):
    """
    Generate a smoothed path from the RRT path.

    Args:
        self: Instance of the RRT class.

    Returns:
        list: Smoothed path coordinates.
    """
    try:
        path_smooth = []
        for index, goal in enumerate(self.path):
            cur = goal
            if cur.parent is not None:
                start = self.goals[index]
                # Draw Trees or Sample points
                lines = []
                while cur.col != start.col or cur.row != start.row:
                    lines.append((cur.col, cur.row))
                    cur = cur.parent
                lines.append((start.col, start.row))
                lines.reverse()
                lines = np.array(lines)
                path_smooth.extend(smooth_path(lines))

        return path_smooth

    except Exception as e:
        print(Fore.RED + str(e))
        traceback.print_exc()
