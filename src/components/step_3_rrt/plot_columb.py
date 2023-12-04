import numpy as np
import matplotlib.pyplot as plt

from src.components import create_folder
from src.components.common import save_files


def friction_force(mu, normal_force):
    """
    Calculate friction force using Coulomb friction model.

    Parameters:
        - mu: Coefficient of friction (between 0 and 1).
        - normal_force: Normal force (weight of the object in static situations).

    Returns:
        Friction force.
    """
    return mu * normal_force


def is_area_traversable(mu, normal_force, threshold_force):
    """
    Determine if an area is traversable based on the Coulomb friction model.

    Parameters:
        - mu: Coefficient of friction (between 0 and 1).
        - normal_force: Normal force (weight of the object in static situations).
        - threshold_force: Friction force threshold for traversability.

    Returns:
        True if the area is traversable, False otherwise.
    """
    friction_force_value = mu * normal_force
    return friction_force_value >= threshold_force


# Parameters
coefficient_of_friction = 0.7
normal_force = 9.8
threshold_friction_force = 5.0

# Generate data for plotting
mu_values = np.linspace(0, 1, 100)
friction_forces = friction_force(mu_values, normal_force)

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(mu_values, friction_forces, label='Friction Force')
plt.axhline(threshold_friction_force, color='r', linestyle='--', label='Threshold Force')

plt.title('Coulomb Friction Model')
plt.xlabel('Coefficient of Friction (\u03BC)')
plt.ylabel('Friction Force')
plt.legend()
plt.grid(True)
name_folder = create_folder.create_folder("../../solutions")

path_solutions = '../../solutions'
plt.savefig(save_files.get_name_to_save_plot(name_folder, 'coloumb', '../../solutions', '.svg'),
            format='svg')

plt.show()
