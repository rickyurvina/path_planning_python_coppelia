import matplotlib.pyplot as plt
import cv2
import os
import pickle

from src.steps import config


def save_image(img):
    """
    Function to save an image.

    Parameters:
    img (array): The image to be saved.

    Returns:
    None
    """
    folder = "../images"
    prefix = "ambiente"
    ext = ".jpg"

    # Check for existing files and set the counter
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1

    # Save the image with the appropriate name
    filename = os.path.join(folder, prefix + str(i) + ext)
    cv2.imwrite(filename, img)
    print("Saved as", filename)


def get_name_to_save_plot(name_folder, prefix, path_folder=config.PATH_FOLDER, ext=".png"):
    """
    Function to get the name to save a plot.

    Parameters:
    name_folder (str): The name of the folder where the plot will be saved.
    prefix (str): The prefix of the file name.
    path_folder (str): The path of the folder.
    ext (str): The extension of the file.

    Returns:
    str: The name to save the plot.
    """
    folder = path_folder + "/" + name_folder
    # Check for existing files and set the counter
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1
    # Save the image with the appropriate name
    filename = os.path.join(folder, prefix + str(i) + ext)
    print("Saved as", filename)
    return filename


def get_name_to_save_excel(name_folder, prefix, path_folder=config.PATH_FOLDER, ext=".xlsx"):
    """
    Function to get the name to save an Excel file.

    Parameters:
    name_folder (str): The name of the folder where the file will be saved.
    prefix (str): The prefix of the file name.
    path_folder (str): The path of the folder.
    ext (str): The extension of the file.

    Returns:
    str: The name to save the Excel file.
    """
    folder = path_folder + "/" + name_folder
    # Check for existing files and set the counter
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1
    # Save the file with the appropriate name
    filename = os.path.join(folder, prefix + str(i) + ext)
    print("Saved as", filename)
    return filename


def get_name_to_save_json(name_folder, prefix, path_folder=config.PATH_FOLDER, ext=".json"):
    """
    Function to get the name to save a JSON file.

    Parameters:
    name_folder (str): The name of the folder where the file will be saved.
    prefix (str): The prefix of the file name.
    path_folder (str): The path of the folder.
    ext (str): The extension of the file.

    Returns:
    str: The name to save the JSON file.
    """
    folder = path_folder + "/" + name_folder
    # Check for existing files and set the counter
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1
    # Save the file with the appropriate name
    filename = os.path.join(folder, prefix + str(i) + ext)
    print("Saved as", filename)
    return filename


def save_workspace(variables, name_folder, path_folder=config.PATH_FOLDER):
    """
    Function to save workspace variables to a pickle file.

    Parameters:
    variables (dict): The variables to be saved.
    name_folder (str): The name of the folder where the file will be saved.
    path_folder (str): The path of the folder.

    Returns:
    None
    """
    folder = path_folder + "/" + name_folder
    if (variables['prefix']):
        prefix = variables['prefix'] + "_variables"
    else:
        prefix = "variables_map"
    ext = ".pickle"

    # Check for existing files and set the counter
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1

    # Save the variables with the appropriate name
    filename = os.path.join(folder, prefix + str(i) + ext)

    with open(filename, 'wb') as f:
        pickle.dump(variables, f)
    print('Variables saved in the file' + filename)
