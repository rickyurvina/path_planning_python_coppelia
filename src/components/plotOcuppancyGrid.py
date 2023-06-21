import matplotlib.pyplot as plt

def plot_occupancy(occupancy_grid):
    plt.figure()
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.plot(0, 0, 'rx')
    plt.xlabel('Eje X (m)')
    plt.ylabel('Eje Y (m)')
    plt.axis('equal')
    plt.show()