# Main and helper function

from PRM import PRM
from src.components import loadFiles

if __name__ == "__main__":
    # Load the map
    start = (350, 190)
    goal = (350, 270)
    # map_array = load_map("WPI_map.jpg", 0.3)
    map_array = loadFiles.load_data_occupancy_grid()['occupancy_grid']

    # Planning class
    PRM_planner = PRM(map_array)

    # Search with PRM
    PRM_planner.sample(n_pts=1000, sampling_method="uniform")
    PRM_planner.search(start, goal, "uniform")
    PRM_planner.sample(n_pts=1000, sampling_method="random")
    PRM_planner.search(start, goal, "random")
    # PRM_planner.sample(n_pts=2000, sampling_method="gaussian")
    # PRM_planner.search(start, goal,"gaussian")
    # PRM_planner.sample(n_pts=20000, sampling_method="bridge")
    # PRM_planner.search(start, goal,"bridge")
