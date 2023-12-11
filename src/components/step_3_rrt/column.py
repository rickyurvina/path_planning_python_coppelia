def is_area_traversable(mu, normal_force, threshold_force):
    """
    Determine if an area is traversable based on the Coulomb friction model.

    Parameters:
        - mu: Coefficient of friction (between 0 and 1).
        - normal_force: Normal force (weight of the object in static situations).
        - threshold_force: Friction force threshold for traversability.

    Returns:
        True if the area is traversable, False otherwise. Coloumb Model of Friction Normal force = peso
    """
    friction_force_value = mu * normal_force
    print("Friction force value:", friction_force_value)
    return friction_force_value >= threshold_force


coefficient_of_friction = 0.7
normal_force = 9.8
threshold_friction_force = 5.0
is_traversable = is_area_traversable(coefficient_of_friction, normal_force, threshold_friction_force)

print("Coefficient of friction:", coefficient_of_friction)
print("Normal force:", normal_force)
print("Threshold friction force:", threshold_friction_force)

if is_traversable:
    print("The area is traversable.")
else:
    print("The area is not traversable.")
