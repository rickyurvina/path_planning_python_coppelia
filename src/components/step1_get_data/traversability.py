import math


def bekker_wong(wheel_radius, sinkage, cohesion, friction_angle, object_friction):
    """
    Calculates the soil resistance force using the Bekker-Wong formula.

    Parameters:
    - wheel_radius: wheel radius (m)
    - sinkage: wheel sinkage into the soil (m)
    - cohesion: soil cohesion (Pa)
    - friction_angle: soil friction angle (in degrees)
    - object_friction: soil friction of object in coppelia [0-1]


    Returns:
    - soil resistance force (N)
    """

    friction_angle_rad = math.radians(friction_angle)
    Rc = (wheel_radius / sinkage) * (cohesion / (object_friction * math.tan(friction_angle_rad)))

    return Rc


def can_traverse_terrain(object_friction, traction_force=400, wheel_radius=0.3, sinkage=0.2, cohesion=50,
                         friction_angle=30,
                         ):
    """
    Determines if a vehicle can traverse a terrain using the Bekker-Wong model.

    Parameters:
    - object_friction: soil friction of object in coppelia [0-1]
    - vertical_force: vertical force applied by the wheel (N)
    - wheel_radius: wheel radius (m)
    - sinkage: wheel sinkage into the soil (m)
    - cohesion: soil cohesion (Pa)
    - friction_angle: soil friction angle (degrees)

    Returns:
    - True if the vehicle can traverse the terrain, False otherwise.
    """
    soil_resistance = bekker_wong(wheel_radius, sinkage, cohesion, friction_angle, object_friction)
    print(f"Soil resistance: {soil_resistance}")
    print(f"Traction force: {traction_force}")
    print(f"Can traverse? {soil_resistance <= traction_force}")
    return soil_resistance, traction_force, soil_resistance <= traction_force


if __name__ == '__main__':
    wheel_radius_1 = 0.3  # Wheel radius (m)
    sinkage_1 = 0.2  # Wheel sinkage into the soil (m)
    cohesion_1 = 50  # Soil cohesion (Pa)
    friction_angle_1 = 30  # Soil friction angle (degrees)
    object_friction = 0.4
    resistance_1 = bekker_wong(wheel_radius_1, sinkage_1, cohesion_1, friction_angle_1, object_friction)
    print(f"Soil resistance 1: {resistance_1} N")
    traction_force_husky_can = 400  # Traction force available on the wheels (N)

    soil_resistance, traction_force, can_traverse_husky_can = can_traverse_terrain(object_friction,
                                                                                   traction_force_husky_can,
                                                                                   wheel_radius_1, sinkage_1,
                                                                                   cohesion_1, friction_angle_1,
                                                                                   )

    print(f"Bekker wong resistance? {soil_resistance}")
    print(f"Traction robot force {traction_force}")
    print(f"Can the robot traverse this terrain? {can_traverse_husky_can}")
