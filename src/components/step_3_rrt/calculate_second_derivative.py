import numpy as np
import matplotlib.pyplot as plt


def calculate_second_derivative(x, y, t):
    dx_dt = np.gradient(x, t)
    dy_dt = np.gradient(y, t)
    d2x_dt2 = np.gradient(dx_dt, t)
    d2y_dt2 = np.gradient(dy_dt, t)
    return d2x_dt2, d2y_dt2


def calculate_smoothness(d2x_dt2, d2y_dt2):
    return np.mean(np.sqrt(d2x_dt2 ** 2 + d2y_dt2 ** 2))


def generate_random_path(t):
    # Genera una trayectoria aleatoria para comparar con una sinusoidal
    np.random.seed(0)
    x = np.cumsum(np.random.randn(len(t)))
    y = np.cumsum(np.random.randn(len(t)))
    return x, y


# Ejemplo de uso
if __name__ == "__main__":
    # Definimos el tiempo
    t = np.linspace(0, 10, 100)

    # Generamos una trayectoria sinusoidal y una aleatoria
    x1, y1 = np.sin(t), np.cos(t)
    x2, y2 = generate_random_path(t)

    # Calculamos las segundas derivadas para cada trayectoria
    d2x_dt2_1, d2y_dt2_1 = calculate_second_derivative(x1, y1, t)
    d2x_dt2_2, d2y_dt2_2 = calculate_second_derivative(x2, y2, t)

    # Calculamos la suavidad de cada trayectoria
    smoothness_1 = calculate_smoothness(d2x_dt2_1, d2y_dt2_1)
    smoothness_2 = calculate_smoothness(d2x_dt2_2, d2y_dt2_2)

    # Imprimimos la suavidad de cada trayectoria
    print("Suavidad de la trayectoria sinusoidal:", smoothness_1)
    print("Suavidad de la trayectoria aleatoria:", smoothness_2)

    # Graficamos las trayectorias
    plt.figure(figsize=(10, 5))
    plt.plot(x1, y1, label='Trayectoria sinusoidal')
    plt.plot(x2, y2, label='Trayectoria aleatoria')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Comparación de Trayectorias')
    plt.grid(True)
    plt.legend()
    plt.show()
