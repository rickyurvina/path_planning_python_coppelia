import tsp
import generateOccupancyGrid
import rrt
import generateRGB


def main():
    ordered_positions = tsp.main()
    occupancy_grid = generateOccupancyGrid.generateOcuppancy();
    rgb = generateRGB.generate_rgb()
    rrt.main_rrt(occupancy_grid, ordered_positions, rgb)

if __name__ == '__main__':
    main()
