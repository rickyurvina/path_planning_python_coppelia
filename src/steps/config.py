# Maximum capacity of the vehicle
VEHICLE_CAPACITIES = 100
# Starting point for the Traveling Salesman Problem (TSP)
START_POINT_TSP = 0

# Number of rows in the grid
NUM_ROWS = 6

# Number of border points generated per row
NUM_POINTS_BORDER_BY_ROW = 1

# Number of center points generated per row
NUM_POINTS_CENTER_BY_ROW = 0

# Rate at which to sample the goal state for RRT
GOAL_SAMPLE_RATE = 0.05

# Minimum number of iterations for RRT
MIN_ITER = 2500

# Maximum number of iterations for RRT
MAX_ITER = 2500

# Radius for extending the RRT tree
RADIUS = 60

# Size of the neighborhood to search for the nearest node
NEIGHBOR_SIZE = 100

# Break the RRT loop if the distance to the goal is less than this value
BREAK_AT = 10

# Method used for RRT (e.g., "INFORMED_RRT_UNICYCLE")
METHOD = "INFORMED_RRT_UNICYCLE"

# Flag to indicate whether the RRT algorithm is executed online
ON_LINE = 0

# Resolution of the grid in the x-direction
RESOLUTION_X = 1000

# Resolution of the grid in the y-direction
RESOLUTION_Y = 1000

# Width of the map/grid
MAP_WIDTH = 15

# Height of the map/grid
MAP_HEIGHT = 15

# Weight factor used in calculations
FACTOR_WEIGHT = 2

# Flag indicating whether a Husky vehicle is used
HUSKY = 1

# Name of the solution folder
SOLUTION = "solution_2049_07052024"

# Folder containing TSP tests
FOLDER_TSP_TESTS = "solution_580_08112023"

# Message template for displaying the number of nodes used to find a path
MESSAGE_NODES = "It took %d nodes to find the current path"

# Message template for displaying the length of a path
MESSAGE_PATH = "The path length is %.2f"

# Message indicating that no path was found
PATH_NO_FOUND = "No path found"

# Identifier for position index
POSITION_INDEX = "position index"

# Message indicating completion
MESSAGE_DONE = "Done"

# Message indicating that plotting is complete
MESSAGE_PLOTTED = "Plotted"

# Limit for area of interest
LIMIT_AOI = 50

# Folder path for saving solutions
PATH_FOLDER = '../solutions'

# Size of the map/grid
SIZE_MAP = 1000

# Scale factor for the map/grid
SCALE_MAP = 15

# Time limit for operations
TIME_LIMIT = 1500

# Flag to indicate whether to draw the RRT tree
DRAW_TREE_RRT = 0

# Number of waypoints
NUM_WAY_POINTS = 6

# Number of boxes
NUM_BOXES = 6

# Number of rows in the map
NUM_ROWS_MAP = 8

# Number of different terrains
NUM_TERRAINS = 5

# Flag to indicate whether to plot the RRT tree
NO_PLOT_RRT = 1

# Flag to indicate whether to draw the safe path
DRAW_SAFE_PATH = 0

# Radius for clearance
CLEARANCE_RADIUS = 20

# Flag to indicate whether to detect friction terrain
DETECT_FRICTION_TERRAIN = True

# Identifier for test reports
TEST_NUMBER_REPORTS = '20240219-9d82c29e-99a1-493f-bd2f-e5f6b81e0332'

# Variables for MySql
MYSQL_HOST = 'localhost'
MYSQL_USER = 'root'
MYSQL_PASSWORD = '12345678'
MYSQL_DATABASE = 'tesis'
