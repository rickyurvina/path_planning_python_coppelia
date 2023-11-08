import os


def remove_empty_folders(root_folder):
    for folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, folder)
        if os.path.isdir(folder_path):
            if not os.listdir(folder_path):  # Check if the folder is empty
                os.rmdir(folder_path)
                print(f"Empty folder removed: {folder_path}")


if __name__ == '__main__':
    main_folder = "../../solutions"
    remove_empty_folders(main_folder)
