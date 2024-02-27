import numpy as np
from matplotlib import pyplot as plt
from src.components.step_3_rrt.path_to_x_y import get_path_coordinates


def calculate_curvature(x, y):
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvature = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt ** 2 + dy_dt ** 2) ** (3 / 2)
    return curvature, d2x_dt2, d2y_dt2


def calculate_curvature_variation(self, x=[], y=[]):
    if len(x) == [] or len(y) == []:
        print("No hay camino")
    else:
        x, y = get_path_coordinates(self)
    path_x_y = np.column_stack((x, y))
    curvature, d2x_dt2, d2y_dt2 = calculate_curvature(x, y)
    smoothness = calculate_smoothness(d2x_dt2, d2y_dt2)
    return curvature, smoothness


def calculate_smoothness(d2x_dt2, d2y_dt2):
    return np.mean(np.sqrt(d2x_dt2 ** 2 + d2y_dt2 ** 2))


def plot_curvature(curvature):
    plt.figure(figsize=(10, 5))
    plt.plot(curvature, label='Curvatura')
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.xlabel('Punto en el camino')
    plt.ylabel('Curvatura')
    plt.title('Variación de la Curvatura')
    plt.grid(True)
    plt.legend()
    plt.show()


# Ejemplo de uso
if __name__ == "__main__":
    # Supongamos que tenemos una lista de coordenadas que representan un camino
    # Aquí hay un ejemplo simple de una curva en forma de S
    x = np.linspace(0, 10, 100)
    y = np.sin(x) + np.cos(x)

    # Calculamos la variación de la curvatura
    curvature = calculate_curvature(x, y)
    variation = calculate_curvature_variation(x, y)
    print("Variación de la curvatura:", variation)
    # Graficar el camino
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, label='Camino')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Camino')
    plt.grid(True)
    plt.legend()
    plt.show()
    plot_curvature(x, y)
