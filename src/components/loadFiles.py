import pickle
from dotenv import load_dotenv
import os

load_dotenv()


def load_data_occupancy_grid():
    with open('../solutions/solution_193_10072023/app_variables1.pickle', 'rb') as f:
        data = pickle.load(f)
    return data


def load_solution_data():
    solution = os.getenv('SOLUTION')
    with open(f'../solutions/{solution}' + '/app_variables1.pickle', 'rb') as f:
        data = pickle.load(f)
    return data


def load_rgb(num):
    with open('../files/workspaces/rgb_variables' + str(num) + '.pickle', 'rb') as f:
        data = pickle.load(f)
    return data
