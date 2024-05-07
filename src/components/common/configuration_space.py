class ConfigurationSpace:
    """
    This class represents the configuration space of a robot.
    """

    def __init__(self, occupancy_grid, robot_radius):
        """
        Initialize the ConfigurationSpace with an occupancy grid and the robot's radius.
        """
        self.occupancy_grid = occupancy_grid
        self.robot_radius = robot_radius
        self.x_min, self.x_max, self.y_min, self.y_max = self.calculate_limits()

    def calculate_limits(self):
        """
        Calculate the limits of the configuration space based on the size of the occupancy grid
        and the physical dimensions of the robot.
        """
        x_min = 0
        x_max = len(self.occupancy_grid[0]) - 1
        y_min = 0
        y_max = len(self.occupancy_grid) - 1
        return x_min, x_max, y_min, y_max

    def is_valid_configuration(self, x, y):
        """
        Check if the configuration (x, y) is valid in the occupancy grid.
        """
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return False  # Outside the limits of the occupancy grid
        if self.occupancy_grid[y][x] == 1:
            return False  # Inside an obstacle
        return True


# Example of use
if __name__ == "__main__":
    # Assume you have an occupancy grid represented as a matrix where 0 is free space and 1 is an obstacle
    occupancy_grid = [
        [0, 0, 0, 1, 1],
        [0, 1, 0, 0, 1],
        [0, 0, 0, 1, 0],
        [1, 0, 1, 1, 0],
        [1, 0, 0, 0, 0],
    ]
    robot_radius = 0.5  # Radius of the unicycle robot
    config_space = ConfigurationSpace(occupancy_grid, robot_radius)

    # Check if a configuration (x, y) is valid
    x = 2
    y = 3
    if config_space.is_valid_configuration(x, y):
        print(f"The configuration ({x}, {y}) is valid.")
    else:
        print(f"The configuration ({x}, {y}) is not valid due to an obstacle or it is outside the space.")
