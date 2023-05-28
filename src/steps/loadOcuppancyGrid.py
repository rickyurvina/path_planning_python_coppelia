import matplotlib.pyplot as plt
import pickle

# Cargar las variables guardadas en el archivo .pickle
with open('../files/variables_map57.pickle', 'rb') as f:
    data = pickle.load(f)
    occupancy_grid = data['occupancy_grid']
    mapa_local = data['mapa_local']

def return_ocuppancy_grid():
    return data

# Mostrar el mapa generado con los rectángulos detectados
# plt.imshow(mapa_local, cmap='gray', origin='lower')
# # plt.imshow(occupancy_grid, origin='lower')
# plt.xlabel('Eje X (m)')
# plt.ylabel('Eje Y (m)')
# plt.axis('equal')
# plt.show()
