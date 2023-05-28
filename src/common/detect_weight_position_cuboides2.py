from src.coppelia import sim
import matplotlib.pyplot as plt

# Establecer la conexión con CoppeliaSim
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    print('No se pudo conectar con CoppeliaSim')
    exit()
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# Obtener el handle del Vision Sensor y de los cuboides
_, vision_sensor_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
num_cuboids = 20  # Definir el número de cuboides que estarán presentes en la escena
cuboid_handles = []
for i in range(num_cuboids):
    _, cuboid_handle = sim.simxGetObjectHandle(clientID, f'Cuboid{i + 1}', sim.simx_opmode_blocking)
    cuboid_handles.append(cuboid_handle)

# Obtener los datos del Vision Sensor
_, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_sensor_handle, 0, sim.simx_opmode_streaming)
while len(image) == 0:
    _, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_sensor_handle, 0, sim.simx_opmode_buffer)

# Obtener la posición y peso de los cuboides
positions = []
weights = []
for cuboid_handle in cuboid_handles:
    _, position = sim.simxGetObjectPosition(clientID, cuboid_handle, -1, sim.simx_opmode_blocking)
    positions.append(position)
    _, mass = sim.simxGetObjectFloatParameter(clientID, cuboid_handle, 3005, sim.simx_opmode_blocking)
    weights.append(mass)
# Graficar los cuboides en una escala de color roja
fig, ax = plt.subplots()
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
for position, weight in zip(positions, weights):
    ax.add_patch(plt.Rectangle((position[0]-0.1, position[1]-0.1), 0.2, 0.2, color=(weight, 0, 0)))
plt.show()

# Cerrar la conexión con CoppeliaSim
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(clientID)
