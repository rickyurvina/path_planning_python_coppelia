import os
import pandas as pd
import json

from src.components import create_folder
from src.components.common.save_files import get_name_to_save_excel
from src.components.step_4_controller.pixels_to_meters import pixels_to_meters
from src.steps import config


def export_to_excel_and_json(path_array, name_folder):
    base_filename = "path_data"  # Nombre base para los archivos
    path_array_meters = pixels_to_meters(path_array)
    df = pd.DataFrame(path_array, columns=['x', 'y'])
    df_2 = pd.DataFrame(path_array_meters, columns=['x', 'y'])
    filename = f"{base_filename}-resolution_pixels"
    filename_2 = f"{base_filename}-resolution_meters"
    file_name_excel = get_name_to_save_excel(name_folder, filename, '../../solutions', ext=".xlsx")
    file_name_excel_2 = get_name_to_save_excel(name_folder, filename_2, '../../solutions', ext=".xlsx")
    file_name_json = get_name_to_save_excel(name_folder, filename, '../../solutions', ext=".json")
    file_name_json_2 = get_name_to_save_excel(name_folder, filename_2, '../../solutions', ext=".json")
    df.to_excel(file_name_excel, index=False)
    df_2.to_excel(file_name_excel_2, index=False)
    path_json = df.to_dict(orient='records')
    path_json_2 = df_2.to_dict(orient='records')
    with open(file_name_json, 'w') as json_file:
        json.dump(path_json, json_file)

    with open(file_name_json_2, 'w') as json_file:
        json.dump(path_json_2, json_file)


if __name__ == '__main__':
    print(create_folder.create_folder())
    path_array = [(0, 0), (1, 1), (2, 2), (3, 3)]
    name_folder = create_folder.create_folder("../../solutions")
    export_to_excel_and_json(path_array, name_folder)
