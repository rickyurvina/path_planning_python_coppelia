import tsp
import generateOccupancyGrid
from src.components.plotOcuppancyGrid import plot_occupancy
import rrt


def main():
    ordered_positions = tsp.main()
    occupancy_grid = generateOccupancyGrid.generateOcuppancy();
    # plot_occupancy(occupancy_grid)
    # print(ordered_positions)
    rrt.main_rrt(occupancy_grid, ordered_positions)


if __name__ == '__main__':
    main()
