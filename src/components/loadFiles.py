import matplotlib.pyplot as plt
import pickle


def load_data_occupancy_grid():
    with open('../files/workspaces/map_variables64.pickle', 'rb') as f:
        data = pickle.load(f)
    return data

def load_solution_data():
    with open('../files/workspaces/solution_variables8.pickle', 'rb') as f:
        data = pickle.load(f)
    return data
