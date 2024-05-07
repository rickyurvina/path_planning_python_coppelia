import datetime
import os

from src.steps import config


def create_folder(path_folder=config.PATH_FOLDER):
    """
    Create a new folder for saving solutions.

    Args:
        path_folder (str): Path to the folder where solutions will be saved. Defaults to config.PATH_FOLDER.

    Returns:
        str: Name of the newly created folder.
    """
    fecha_actual = datetime.datetime.now().strftime("%d%m%Y")

    # Path to the folder where solutions will be saved
    ruta_solutions = path_folder

    # Get the number of the last solution in the directory
    num_soluciones = sum(1 for _ in os.listdir(ruta_solutions) if os.path.isdir(os.path.join(ruta_solutions, _)))

    # Create the base name of the new folder
    nombre_base = f"solution_{num_soluciones + 1}_{fecha_actual}"

    # Check if a folder with the base name already exists
    nombre_carpeta = nombre_base
    contador = 1
    while os.path.exists(os.path.join(ruta_solutions, nombre_carpeta)):
        nombre_carpeta = f"{nombre_base}_{contador}"
        contador += 1

    parent_folder = path_folder

    new_folder_path = os.path.join(parent_folder, nombre_carpeta)
    os.mkdir(new_folder_path)
    return nombre_carpeta


if __name__ == '__main__':
    print(create_folder())
