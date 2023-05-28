import numpy as np
import cv2
import matplotlib.pyplot as plt
import pickle

# Cargar las variables guardadas en el archivo .pickle
with open('../files/variables_map5.pickle', 'rb') as f:
    data = pickle.load(f)
    occupancy_grid = data['occupancy_grid']
    mapa_local = data['mapa_local']

# Definir los límites de umbralización para detectar los rectángulos
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([50, 50, 50])

# Convertir el mapa local en una imagen de OpenCV
mapa_local_cv = cv2.cvtColor(mapa_local, cv2.COLOR_GRAY2RGB)

# Aplicar la umbralización para detectar los rectángulos
mask = cv2.inRange(mapa_local_cv, lower_bound, upper_bound)
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Dibujar los contornos de los rectángulos en el mapa local
cv2.drawContours(mapa_local_cv, contours, -1, (0, 255, 0), 2)

# Mostrar el mapa generado con los rectángulos detectados
plt.imshow(mapa_local_cv, origin='lower')
plt.xlabel('Eje X (m)')
plt.ylabel('Eje Y (m)')
plt.axis('equal')
plt.show()
