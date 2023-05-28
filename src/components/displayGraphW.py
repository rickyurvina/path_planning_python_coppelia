import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import cv2
import os
from numpy.distutils.fcompiler import none

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
def plotsPositions(route=none, weights=none, positions=none):
    jet = cm.get_cmap('jet')
    scalar_map = cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=max(weights)), cmap=jet)
    # Graficar los cuboides en una escala de color JET y el camino óptimo en amarillo
    fig, ax = plt.subplots()
    ax.set_xlim([1, 6])
    ax.set_ylim([1, 7])
    max_weight = np.max(weights)
    cmap = plt.get_cmap('jet')  # obtener el colormap Jet
    norm = plt.Normalize(vmin=0, vmax=max_weight)  # normalizar los pesos
    ordered_positions = []
    for row in range(len(route)):
        ordered_positions.append(positions[route[row]])
    for position, weight in zip(positions, weights):
        color = cmap(norm(weight))
        circle = plt.Circle((position[0], position[1]), 0.10, color=color)
        ax.add_artist(circle)
    plt.colorbar(scalar_map, label='Peso')

    for i in range(len(ordered_positions) - 1):
        ax.arrow(ordered_positions[i][0], ordered_positions[i][1],
                 ordered_positions[i + 1][0] - ordered_positions[i][0],
                 ordered_positions[i + 1][1] - ordered_positions[i][1],
                 color='red',linestyle='dashed', length_includes_head=True, head_width=0.1)
    total_distance = 0
    for i in range(len(ordered_positions) - 1):
        dist = np.linalg.norm(np.array(ordered_positions[i]) - np.array(ordered_positions[i + 1]))
        total_distance += dist

    # Crear la leyenda con la distancia total
    legend_text = f"Distancia total: {total_distance:.2f}"
    ax.text(0.05, -0.1, legend_text, transform=ax.transAxes, fontsize=10)
    folder = "../images/plots"
    prefix = "ruta_optima"
    ext = ".png"
    # Verificar la existencia de archivo y establecer el contador
    i = 1
    while os.path.exists(os.path.join(folder, prefix + str(i) + ext)):
        i += 1
    # Guardar la imagen con el nombre adecuado
    filename = os.path.join(folder, prefix + str(i) + ext)
    plt.savefig(filename, dpi=500)
    print("Guardado como", filename)
    plt.title("TSP Ordered Harvesting postions")
    plt.show()

