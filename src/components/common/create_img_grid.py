from PIL import Image
import os

from src.steps import config


def create_image_grid(folder_path, output_path, number_of_images):
    """
    Create an image grid from the images in the specified folder.
    """
    # Get the list of files in the folder
    files = os.listdir(folder_path)

    # Filter files with .png extension
    image_files = [file for file in files if file.lower().endswith(('.png'))]

    # Check if there are enough images
    if len(image_files) < number_of_images:
        raise ValueError("At least 4 images are required to create the grid.")

    # Create an empty image with the size of the grid
    grid_size = (1, number_of_images)
    grid_image = Image.new('RGB', (2 * 200, 2 * 200))

    for i in range(number_of_images):
        image_path = os.path.join(folder_path, image_files[i])
        img = Image.open(image_path)

        # Paste the image into the grid
        x = i % 2 * 200
        y = i // 2 * 200
        grid_image.paste(img, (x, y))

    # Save the image grid
    grid_image.save(output_path)
    grid_image.show()


def count_png_images(folder_path):
    """
    Count the number of PNG images in the specified folder.
    """
    # Get the list of files in the folder
    files = os.listdir(folder_path)

    # Filter files with .png extension
    png_files = [file for file in files if file.lower().endswith('.png')]

    # Return the number of PNG files
    return len(png_files)


if __name__ == '__main__':
    solution = config.FOLDER_TSP_TESTS
    folder = "../" + config.PATH_FOLDER + '/' + solution
    number_of_images = count_png_images(folder)
    create_image_grid(folder, folder + '/grid_image.png', number_of_images)
