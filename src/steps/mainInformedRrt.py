from RRT_INFORMED import RRT_INFORMED
from src.components.shiftPositions import shift_positions
from src.components import loadFiles


def main_informed_rrt(map_array, ordered_positions, name_folder):

    ordered_transformed = shift_positions(ordered_positions)
    RRT_planner = RRT_INFORMED(map_array, ordered_transformed, name_folder)

    return RRT_planner.informed_RRT_star(n_pts=5000)

if __name__ == "__main__":
    # Load the map
    data = loadFiles.load_data_occupancy_grid()
    occupancy_grid = data['occupancy_grid']
    ordered_positions = data['ordered_positions']
    main_informed_rrt(occupancy_grid, ordered_positions)
