from PIL import Image
import os

from src.steps import config


def create_image_grid(folder_path, output_path, number_of_images):
    # Obtener la lista de archivos en la carpeta
    files = os.listdir(folder_path)

    # Filtrar archivos con extensión .png
    image_files = [file for file in files if file.lower().endswith(('.png'))]

    # Verificar si hay suficientes imágenes
    if len(image_files) < number_of_images:
        raise ValueError("Se requieren al menos 4 imágenes para crear la cuadrícula.")

    # Crear una imagen vacía con el tamaño de la cuadrícula
    grid_size = (1, number_of_images)
    grid_image = Image.new('RGB', (2 * 200, 2 * 200))

    for i in range(number_of_images):
        image_path = os.path.join(folder_path, image_files[i])
        img = Image.open(image_path)

        # Pegar la imagen en la cuadrícula
        x = i % 2 * 200
        y = i // 2 * 200
        grid_image.paste(img, (x, y))

    # Guardar la cuadrícula de imágenes
    grid_image.save(output_path)
    grid_image.show()


def count_png_images(folder_path):
    # Obtener la lista de archivos en la carpeta
    files = os.listdir(folder_path)

    # Filtrar archivos con extensión .png
    png_files = [file for file in files if file.lower().endswith('.png')]

    # Retornar el número de archivos PNG
    return len(png_files)


if __name__ == '__main__':
    solution = config.FOLDER_TSP_TESTS
    folder = "../" + config.PATH_FOLDER + '/' + solution
    number_of_images = count_png_images(folder)
    create_image_grid(folder, folder + '/grid_image.png', number_of_images)
