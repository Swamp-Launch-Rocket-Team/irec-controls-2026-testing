import rocketpy
import constants.rocket_c as r

env = rocketpy.Environment(
    latitude=r.sim_location[0], longitude=r.sim_location[1], elevation=r.sim_location[2]
)

env.set_date(
    (r.sim_date.year, r.sim_date.month, r.sim_date.day, 12)
)

env.set_atmospheric_model(type="Forecast", file="GFS")

motor = rocketpy.SolidMotor(
    thrust_source="simulation/AeroTech_M2500T.eng",
    dry_mass=r.motor_mass,
    dry_inertia=r.motor_dry_inertia,
    nozzle_radius=r.motor_radius,
    grain_number=r.grain_number,
    grain_density=r.grain_density,
    grain_outer_radius=r.grain_outer_radius,
    grain_initial_inner_radius=r.grain_initial_inner_radius,
    grain_initial_height=r.grain_initial_height,
    grain_separation=r.grain_separation,
    grains_center_of_mass_position=r.grains_com_position,
    center_of_dry_mass_position=r.motor_com_position
)

dino = rocketpy.Rocket(
    radius= r.rocket_radius,
    mass=r.rocket_mass,
    inertia=r.rocket_inertia,
    power_off_drag="constants/CD Power Off.CSV",
    power_on_drag="constants/CD Power On.CSV",
    center_of_mass_without_motor=r.rocket_com,
    coordinate_system_orientation="nose_to_tail"
)

dino.add_motor(motor, r.rocket_length)

dino.add_trapezoidal_fins(
    n=r.fins_n,
    root_chord=r.fins_root_chord,
    tip_chord=r.fins_tip_chord,
    span=r.fins_span,
    position=r.fins_position,
    cant_angle=r.fins_cant_angle,
    airfoil=r.fins_airfoil
)

dino.add_nose(
    length=r.nose_length, kind=r.nose_kind, position=r.nose_position
)

dino.add_tail(
    top_radius=r.tail_top_radius, 
    bottom_radius=r.tail_bottom_radius, 
    length=r.tail_length, 
    position=r.tail_position
)

test_flight = rocketpy.Flight(
    rocket=dino, environment=env, rail_length=5.2, inclination=85, heading=0
)

