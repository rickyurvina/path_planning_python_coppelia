from src.components.step_3_rrt.path_to_file import export_to_excel_and_json


def get_path_coordinates(self):
    # Inicializar listas para almacenar las coordenadas x e y
    x_positions = []
    y_positions = []

    # Recorrer el árbol de nodos y almacenar las coordenadas
    if self.name_method == "IRRT":
        print("El método es IRRT")
        x_positions = [point[0] for point in self.path]
        y_positions = [point[1] for point in self.path]
    else:
        current_node = self.path[0]
        while current_node is not None:
            x_positions.append(current_node.col)
            y_positions.append(current_node.row)
            current_node = current_node.parent
        x_positions.reverse()
        y_positions.reverse()

    x_y_positions = list(zip(x_positions, y_positions))
    export_to_excel_and_json(x_y_positions, self.name_folder)

    return x_positions, y_positions
