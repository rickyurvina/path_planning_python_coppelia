import datetime
import os

def create_folder():
    fecha_actual = datetime.datetime.now().strftime("%d%m%Y")

    # Ruta de la carpeta donde se guardarán las soluciones
    ruta_solutions = '../solutions'

    # Obtener el número de la última solución en el directorio
    num_soluciones = sum(1 for _ in os.listdir(ruta_solutions) if os.path.isdir(os.path.join(ruta_solutions, _)))

    # Crear el nombre base de la nueva carpeta
    nombre_base = f"solution_{num_soluciones + 1}_{fecha_actual}"

    # Comprobar si ya existe una carpeta con el nombre base
    nombre_carpeta = nombre_base
    contador = 1
    while os.path.exists(os.path.join(ruta_solutions, nombre_carpeta)):
        nombre_carpeta = f"{nombre_base}_{contador}"
        contador += 1

    parent_folder = "../solutions"  # Reemplaza con la ruta específica deseada

    new_folder_path = os.path.join(parent_folder, nombre_carpeta)
    os.mkdir(new_folder_path)
    return nombre_carpeta