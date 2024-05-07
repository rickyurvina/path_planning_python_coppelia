import pickle
import sys

from src.steps import config

# Add the RRT directory to the system path
sys.path.append('../../RRT')
data = {}


def load_data_occupancy_grid():
    """
    Load occupancy grid data from a pickle file.
    """
    with open(config.PATH_FOLDER + '/solution_193_10072023/app_variables1.pickle', 'rb') as f:
        unpickler = pickle.Unpickler(f)
        # If file is not empty, data will be equal to the value unpickled
        data = unpickler.load()
    return data


def load_solution_data(solution_folder=config.PATH_FOLDER):
    """
    Load solution data from a pickle file.
    """
    with open(f'../' + solution_folder + "/" + config.SOLUTION + '/app_variables1.pickle', 'rb') as f:
        unpickler = pickle.Unpickler(f)
        # If file is not empty, data will be equal to the value unpickled
        data = unpickler.load()
    return data


def load_file_excel(solution_folder=config.PATH_FOLDER):
    """
    Load data from a pickle file.
    """
    with open(f'../' + solution_folder + "/" + config.SOLUTION + '/app_variables1.pickle', 'rb') as f:
        unpickler = pickle.Unpickler(f)
        # If file is not empty, data will be equal to the value unpickled
        data = unpickler.load()
    return data


def load_rgb(num):
    """
    Load RGB data from a pickle file.
    """
    with open('../files/workspaces/rgb_variables' + str(num) + '.pickle', 'rb') as f:
        data = pickle.load(f)
    return data


if __name__ == '__main__':
    # Load solution data and print it
    data = load_solution_data()
    print(data)
