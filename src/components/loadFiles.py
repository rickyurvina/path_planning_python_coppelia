import pickle


def load_data_occupancy_grid():
    with open('../solutions/solution_193_10072023/app_variables1.pickle', 'rb') as f:
        data = pickle.load(f)
    return data


def load_solution_data():
    with open('../solutions/solution_195_10072023/app_variables1.pickle', 'rb') as f:
        data = pickle.load(f)
    return data


def load_rgb(num):
    with open('../files/workspaces/rgb_variables' + str(num) + '.pickle', 'rb') as f:
        data = pickle.load(f)
    return data
