import os


def remove_empty_folders(root_folder):
    """
    Remove all empty folders in the specified root folder.
    """
    # Iterate over all items in the root folder
    for folder in os.listdir(root_folder):
        # Construct the full path to the item
        folder_path = os.path.join(root_folder, folder)
        # Check if the item is a folder
        if os.path.isdir(folder_path):
            # Check if the folder is empty
            if not os.listdir(folder_path):
                # Remove the folder
                os.rmdir(folder_path)
                print(f"Empty folder removed: {folder_path}")


if __name__ == '__main__':
    # Define the main folder
    main_folder = "../../solutions"
    # Call the function to remove all empty folders in the main folder
    remove_empty_folders(main_folder)
