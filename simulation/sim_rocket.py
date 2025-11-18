import rocketpy
import constants.rocket_c as r
from gnc.gnc import GNC
from noisy_ahrs import Noisy_AHRS
from noisy_altimeter import Noisy_Altimeter
from actuator_plant import Airbrake

class Rocket:
    def __init__(self):
        self.env = rocketpy.Environment(
            latitude=r.sim_location[0], longitude=r.sim_location[1], elevation=r.sim_location[2]
        )
        self.env.set_atmospheric_model(type="Forecast", file="GFS")

        self.motor = rocketpy.SolidMotor(
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

        self.dino = rocketpy.Rocket(
            radius= r.rocket_radius,
            mass=r.rocket_mass,
            inertia=r.rocket_inertia,
            power_off_drag="constants/CD Power Off.CSV",
            power_on_drag="constants/CD Power On.CSV",
            center_of_mass_without_motor=r.rocket_com,
            coordinate_system_orientation="nose_to_tail"
        )

        self.dino.add_motor(self.motor, r.rocket_length)

        self.dino.add_trapezoidal_fins(
            n=r.fins_n,
            root_chord=r.fins_root_chord,
            tip_chord=r.fins_tip_chord,
            span=r.fins_span,
            position=r.fins_position,
            cant_angle=r.fins_cant_angle,
            airfoil=r.fins_airfoil
        )

        self.dino.add_nose(
            length=r.nose_length, kind=r.nose_kind, position=r.nose_position
        )

        self.dino.add_tail(
            top_radius=r.tail_top_radius, 
            bottom_radius=r.tail_bottom_radius, 
            length=r.tail_length, 
            position=r.tail_position
        )

        self.air_brakes = self.dino.add_air_brakes(
            drag_coefficient_curve="constants\CD Airbrake.csv",
            controller_function=self.get_airbrake,
            sampling_rate=10,
            reference_area=None,
            clamp=True,
            initial_observed_variables=[0, 0, 0],
            override_rocket_drag=False,
            name="Air Brakes",
        )

        self.ahrs = Noisy_AHRS()
        self.altimeter = Noisy_Altimeter()
        self.actuator = Airbrake()

        self.gnc = None

        self.time = 0
        self.loop_number = 0
        self.g_frequency = 4
        self.c_frequency = 50
        self.nu_frequency = 20
        self.np_frequency = 200

    def call_gnc(self, y):
        if self.loop_number == 0:
            self.gnc = GNC()
    
    def get_current_actuation(self):
        pass

    def start_start_sim(self):
        pass

    def plot_results(self):
        pass
    


