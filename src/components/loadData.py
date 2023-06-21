import matplotlib.pyplot as plt
import pickle


def load_data_occupancy_grid():
    with open('../files/workspaces/variables_map75.pickle', 'rb') as f:
        data = pickle.load(f)
    return data

def load_solution_data():
    with open('../files/workspaces/solution_variables3.pickle', 'rb') as f:
        data = pickle.load(f)
    return data
