import numpy as np
import matplotlib.pyplot as plt

# Tamaño del occupancy grid
rows, cols = 10, 10

# Crea un occupancy grid con valores iniciales (pueden ser ceros)
occupancy_grid = np.ones((rows, cols))

# Establece valores diferentes para ciertos objetos
# Supongamos que el objeto en la posición (2, 3) tiene un valor de 0.3
occupancy_grid[2, 3] = 0.3

# Supongamos que el objeto en la posición (5, 7) tiene un valor de 0.4
occupancy_grid[5, 7] = 0.4

# Visualiza el occupancy grid con escala de grises
plt.imshow(occupancy_grid, cmap='gray', vmin=0, vmax=1)
plt.colorbar()
plt.title('Occupancy Grid')
plt.show()
