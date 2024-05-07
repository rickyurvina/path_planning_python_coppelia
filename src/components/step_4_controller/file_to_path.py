import pandas as pd

from src.steps import config


def load_excel_to_array(solution_folder='../solutions'):
    """
    Load data from an Excel file into an array of positions.

    Args:
        solution_folder (str): Path to the folder containing the solution files. Default is '../solutions'.

    Returns:
        list: Array of positions.
    """
    # Load the Excel file into a pandas DataFrame
    file_name = f'../{solution_folder}/{config.SOLUTION}/path_meters1.xlsx'
    df = pd.read_excel(file_name)

    # Extract the first two columns (x and y positions)
    x_column = df.iloc[:, 0]  # First column
    y_column = df.iloc[:, 1]  # Second column

    # Convert the columns to lists and combine them into an array of positions
    path_array = [(x, y) for x, y in zip(x_column, y_column)]

    return path_array


if __name__ == '__main__':
    path_array = load_excel_to_array()
    print("Positions in x and y:", path_array)
