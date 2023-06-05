import pickle
import matplotlib.pyplot as plt

# Cargar las variables guardadas en el archivo .pickle
with open('../files/workspaces/variables_map4.pickle', 'rb') as f:
    data = pickle.load(f)
    mapa_local = data['mapa_local']

# Mostrar el mapa generado
plt.imshow(mapa_local, cmap='gray', origin='lower')
plt.plot(0, 0, 'rx')
plt.xlabel('Eje X (cm)')
plt.ylabel('Eje Y (cm)')
plt.plot(100, 120, 'ro')
plt.plot(1000, 630, 'bo')
plt.axis('equal')
plt.show()
