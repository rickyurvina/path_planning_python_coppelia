import matplotlib.pyplot as plt
import cv2
import os
import pickle

from src.steps import config


def save_image(img):
    folder = "../images"
    prefix = "ambiente"
    ext = ".jpg"

    # Verificar la existencia de archivo y establecer el contador
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1

    # Guardar la imagen con el nombre adecuado
    filename = os.path.join(folder, prefix + str(i) + ext)
    cv2.imwrite(filename, img)
    print("Guardado como", filename)


def get_name_to_save_plot(name_folder, prefix, path_folder=config.PATH_FOLDER):
    folder = path_folder + "/" + name_folder
    ext = ".png"
    # Verificar la existencia de archivo y establecer el contador
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1
    # Guardar la imagen con el nombre adequacy
    filename = os.path.join(folder, prefix + str(i) + ext)
    print("Guardado como", filename)
    return filename


def save_workspace(variables, name_folder, path_folder=config.PATH_FOLDER):
    folder = path_folder + "/" + name_folder
    if (variables['prefix']):
        prefix = variables['prefix'] + "_variables"
    else:
        prefix = "variables_map"
    ext = ".pickle"

    # Verificar la existencia de archivo y establecer el contador
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1

    # Guardar la imagen con el nombre adecuado
    filename = os.path.join(folder, prefix + str(i) + ext)

    with open(filename, 'wb') as f:
        pickle.dump(variables, f)
    print('Variables guardadas en el archivo' + filename)
