import math


def bekker_wong(wheel_radius, sinkage, cohesion, friction_angle, object_friction):
    """
    Function to calculate the soil resistance force using the Bekker-Wong formula.

    Parameters:
    wheel_radius (float): The radius of the wheel in meters.
    sinkage (float): The sinkage of the wheel into the soil in meters.
    cohesion (float): The cohesion of the soil in Pascal.
    friction_angle (float): The friction angle of the soil in degrees.
    object_friction (float): The friction of the object in Coppelia, ranging from 0 to 1.

    Returns:
    float: The soil resistance force in Newton.
    """
    friction_angle_rad = math.radians(friction_angle)
    Rc = (wheel_radius / sinkage) * (cohesion / (object_friction * math.tan(friction_angle_rad)))

    return Rc


def can_traverse_terrain(object_friction, traction_force=400, wheel_radius=0.3, sinkage=0.2, cohesion=50,
                         friction_angle=30):
    """
    Function to determine if a vehicle can traverse a terrain using the Bekker-Wong model.

    Parameters:
    object_friction (float): The friction of the object in Coppelia, ranging from 0 to 1.
    traction_force (float): The traction force available on the wheels in Newton. Default is 400.
    wheel_radius (float): The radius of the wheel in meters. Default is 0.3.
    sinkage (float): The sinkage of the wheel into the soil in meters. Default is 0.2.
    cohesion (float): The cohesion of the soil in Pascal. Default is 50.
    friction_angle (float): The friction angle of the soil in degrees. Default is 30.

    Returns:
    bool: True if the vehicle can traverse the terrain, False otherwise.
    """
    soil_resistance = bekker_wong(wheel_radius, sinkage, cohesion, friction_angle, object_friction)

    return soil_resistance <= traction_force


def transform_value(value):
    """
    Function to transform a value to ensure it is within the range [0, 1].

    Parameters:
    value (float): The value to be transformed.

    Returns:
    float: The transformed value.
    """
    value = max(0, min(1, value))
    transformed_value = 1 - value

    return transformed_value


if __name__ == '__main__':
    wheel_radius_1 = 0.3  # Wheel radius (m)
    sinkage_1 = 0.2  # Wheel sinkage into the soil (m)
    cohesion_1 = 50  # Soil cohesion (Pa)
    friction_angle_1 = 30  # Soil friction angle (degrees)
    object_friction = 0.6
    resistance_1 = bekker_wong(wheel_radius_1, sinkage_1, cohesion_1, friction_angle_1, object_friction)
    print(f"Soil resistance 1: {resistance_1} N")
    traction_force_husky_can = 400  # Traction force available on the wheels (N)

    soil_resistance = can_traverse_terrain(object_friction,
                                           traction_force_husky_can,
                                           wheel_radius_1, sinkage_1,
                                           cohesion_1, friction_angle_1,
                                           )
    print(f"Can the vehicle traverse the terrain? {soil_resistance}")
