import numpy as np
from atmosphere import AtmosphereModel
import constants.rocket_c as r
import constants.atmosphere_c as a

class DynamicsModel:
    def __init__(self, atmosphere: AtmosphereModel):
        self.cla = r.rocket_cla
        self.cd_mach= r.rocket_cd_mach
        self.cd_airbrake = r.rocket_cd_airbrake
        self.atmosphere = atmosphere

    def get_wind(self, state):
        v = np.array([state[2], state[3]])
        w1 = v / np.linalg.norm(v)
        w2 = np.array([-w1[1], w1[0]])
        return w1, w2
    
    def get_dyn_p(self, state, v_mag):
        return 0.5 * self.atmosphere.get_d(state[2]) * v_mag**2

    def get_v_mag(self, state):
        return state[1]**2 + state[3]**2
    
    def get_mach(self, v_mag, c):
        return v_mag / c

    def get_base_cd(self, mach):
        return r.rocket_cd_mach[0] + r.rocket_cd_mach[1] * mach + r.rocket_cd_mach[2] * mach**2
    
    def get_airbrake_cd(self, U):
        return r.rocket_cd_airbrake[0] + r.rocket_cd_airbrake[1] * U
    
    def get_cd(self, base_cd, airbrake_cd):
        return base_cd * airbrake_cd
        
    def get_drag(self, dyn_p, cd):
        return cd * r.rocket_refa * dyn_p
    
    def get_aoa(self, state):
        return r.rocket_aoa_vel[0] + r.rocket_aoa_vel[1] * state[1] + r.rocket_aoa_vel[2] * state[3]
    
    def get_cl(self, aoa):
        return aoa * self.cla
    
    def get_lift(self, dyn_p, cl):
        return cl * r.rocket_refa * dyn_p
    
    def get_actuator_derivative(self, U_command, U_actual):
        return (U_command - U_actual) / r.servo_tao

    
    def get_state_derivative(self, state, input):
        w1, w2 = self.get_wind(state)
        
        c = self.atmosphere.get_c(state[2])
        v_mag = self.get_v_mag(state)
        mach = self.get_mach(v_mag, c)
        dyn_p = self.get_dyn_p(state, v_mag)

        base_cd = self.get_base_cd(mach)
        airbrake_cd = self.get_airbrake_cd(state[4])
        cd = self.get_cd(base_cd, airbrake_cd)
        drag = -self.get_drag(dyn_p, cd) * w1

        aoa = self.get_aoa(state)
        cl = self.get_cl(aoa)
        lift = self.get_lift(dyn_p, cl) * w2

        gravity = -a.gravity * r.rocket_mass * np.array([0, 1])

        net_force = drag + lift + gravity
        net_acceleration = net_force / r.rocket_mass

        U_rate = self.get_actuator_derivative(state[4], input[0])

        return np.array([
            state[1],
            net_acceleration[0],
            state[3],
            net_acceleration[1],
            U_rate
        ])

    def get_linearized(self, state_0, input_0, e=1e-6):
        #forward finite difference approx        
        state_derivative_0 = self.get_state_derivative(state_0, input_0)

        J_state = np.zeros((5, 5))
        J_input = np.zeros((1, 5))

        for i in range(5):
            state_i = state_0.copy()
            state_i[i] += e

            state_partial_i = (self.get_state_derivative(state_i, input_0) - state_derivative_0) / e
            J_state[i] = state_partial_i
        
        for i in range(1):
            input_i = state_0.copy()
            input_i[i] += e

            input_partial_i = (self.get_state_derivative(state_0, input_i) - state_derivative_0) / e
            J_input[i] = input_partial_i
        
        return J_state, J_input
    
    def get_sensor_output(self, state):
        return np.array([state[2], np.atan2(state[3], state[1]) + self.get_aoa(state), state[4]])
    
    def get_linearized_output(self, state_0, e=1e-6):
        #forward finite difference approx        
        output_0 = self.get_sensor_output(state_0)

        J_output = np.zeros((5, 3))

        for i in range(5):
            output_i = state_0.copy()
            output_i[i] += e

            output_partial_i = (self.get_sensor_output(output_i) - output_0) / e
            J_output[i] = output_partial_i
        return J_output
    
    def get_kalman_A(self, dt):
        #regular matrix, assumes constant accel
        return np.array([
            [1, dt, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, dt, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, -dt / r.servo_tao]
        ])
    
    def get_kalman_B(self, dt):
        #input augmented with accelerometer readings
        return np.array([
            [0, 0.5*dt**2, 0],
            [0, dt, 0],
            [0, 0, 0.5*dt**2],
            [0, 0, dt],
            [dt / r.servo_tao, 0, 0]
        ])
    
    def get_kalman_C(self, state_0):
        #standard shit
        return self.get_linearized_output(state_0).T
