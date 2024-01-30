import numpy as np
import matplotlib.pyplot as plt
from src.components.step_3_rrt.find_nearest_obstacle_on_map import find_nearest_obstacle_on_map


def find_nearest_obstacle(occupancy_grid, point):
    x_idx = point.row
    y_idx = point.col
    point = np.array([x_idx, y_idx])
    nearest_obstacle = find_nearest_obstacle_on_map(occupancy_grid, x_idx, y_idx)
    coordinate_difference = point - nearest_obstacle
    distance = np.linalg.norm(coordinate_difference)
    return nearest_obstacle, distance


def main():
    rows, columns = 10, 10
    occupancy_grid = np.random.choice([0, 1], size=(rows, columns), p=[0.2, 0.8])
    free_x, free_y = np.where(occupancy_grid == 0)
    random_point = (free_x[np.random.choice(len(free_x))], free_y[np.random.choice(len(free_y))])
    nearest_obstacle, distance = find_nearest_obstacle(occupancy_grid, random_point)
    print("Occupancy Grid Map:")
    print(occupancy_grid)
    print("\nRandom Point:", random_point)
    print("Coordinates of Nearest Obstacle:", nearest_obstacle)
    print("Distance to Nearest Obstacle:", distance)
    plt.imshow(occupancy_grid, cmap='gray', origin='upper')
    plt.plot(random_point[1], random_point[0], 'go', label='Random Point')
    plt.plot(nearest_obstacle[1], nearest_obstacle[0], 'ro', label='Nearest Obstacle')
    plt.plot([random_point[1], nearest_obstacle[1]], [random_point[0], nearest_obstacle[0]], 'r-',
             label='Distance')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
