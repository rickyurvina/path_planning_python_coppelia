import math


def coulomb_friction(normal_force, friction_coefficient, traction_force=400):
    """
    Calculates the frictional force using the Coulomb friction model.

    Parameters:
    - normal_force: normal force between surfaces (N)
    - friction_coefficient: coefficient of friction (unitless)

    Returns:
    - frictional force (N)
    """
    frictional_force = friction_coefficient * normal_force
    return frictional_force <= traction_force


if __name__ == '__main__':
    normal_force = 50  # Normal force (N)
    friction_coefficient = 0.3  # Coefficient of friction (unitless)

    frictional_force = coulomb_friction(normal_force, friction_coefficient)
    print(f"Frictional force: {frictional_force} N")
