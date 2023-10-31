import pickle
import sys

sys.path.append('../RRT')
data = {}


def load_data_occupancy_grid():
    with open('../solutions/solution_193_10072023/app_variables1.pickle', 'rb') as f:
        unpickler = pickle.Unpickler(f)
        # if file is not empty scores will be equal
        # to the value unpickled
        data = unpickler.load()
    return data


def load_solution_data():
    with open(f'../solutions/solution_701_30102023' + '/app_variables1.pickle', 'rb') as f:
        unpickler = pickle.Unpickler(f)
        # if file is not empty scores will be equal
        # to the value unpickled
        data = unpickler.load()
    return data


def load_rgb(num):
    with open('../files/workspaces/rgb_variables' + str(num) + '.pickle', 'rb') as f:
        data = pickle.load(f)
    return data
