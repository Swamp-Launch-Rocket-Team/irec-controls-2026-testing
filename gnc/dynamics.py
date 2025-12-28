import numpy as np
from gnc.atmosphere import AtmosphereModel
import constants.rocket_c as r
import constants.atmosphere_c as a
import constants.gnc_c as g

class DynamicsModel:
    def __init__(self, atmosphere: AtmosphereModel):
        self.cd_mach= r.rocket_cd_mach
        self.cd_airbrake = r.rocket_cd_airbrake
        self.atmosphere = atmosphere

    def get_wind(self, state, e=0.00001):
        v = np.array([state[1], state[3]])
        w1 = v / (np.linalg.norm(v) + e)
        w2 = np.array([-w1[1], w1[0]])
        return w1, w2
    
    def get_dyn_p(self, state, v_mag):
        density = self.atmosphere.get_d(state[2])
        return 0.5 * density * v_mag**2

    def get_v_mag(self, state):
        return np.sqrt(state[1]**2 + state[3]**2)
    
    def get_mach(self, v_mag, c):
        return v_mag / c

    def get_base_cd(self, mach):
        base_cd = 0.0

        for i in range(len(r.rocket_cd_mach)):
            base_cd += r.rocket_cd_mach[i] * mach ** i
        
        return base_cd
    
    def get_airbrake_cd(self, nav_input, drag_state):
        return drag_state[0] + drag_state[1] * nav_input
    
    def get_cd(self, base_cd, airbrake_cd):
        return base_cd * airbrake_cd
        
    def get_drag(self, dyn_p, cd):
        return cd * r.rocket_refa * dyn_p
    
    def get_cl(self, drag_state, vz):
        cl = 0.0

        for i in range(len(r.rocket_cl_vz)):
            cl += drag_state[i + 2] * vz ** i
        
        return cl
    
    def get_lift(self, dyn_p, cl):
        return cl * r.rocket_refa * dyn_p
    
    def get_actuator_derivative(self, U_command, U_actual):
        return (U_command - U_actual) / r.servo_tao
    
    def get_nav_state_derivative(self, nav_state, a_state, drag_state):
        w1, w2 = self.get_wind(nav_state)
        
        c = self.atmosphere.get_c(nav_state[2])
        v_mag = self.get_v_mag(nav_state)
        mach = self.get_mach(v_mag, c)
        dyn_p = self.get_dyn_p(nav_state, v_mag)

        base_cd = self.get_base_cd(mach)
        airbrake_cd = self.get_airbrake_cd(a_state, drag_state)
        cd = self.get_cd(base_cd, airbrake_cd)
        cl = self.get_cl(drag_state, nav_state[3])

        drag = -self.get_drag(dyn_p, cd) * w1
        lift = self.get_lift(dyn_p, cl) * w2

        gravity = -a.gravity * r.coast_mass * np.array([0, 1])

        net_force = drag + gravity + lift
        net_acceleration = net_force / r.coast_mass

        return np.array([
            nav_state[1],
            net_acceleration[0],
            nav_state[3],
            net_acceleration[1]
        ])
    

    def get_z_apogee(self, nav_state, drag_state, a_state=g.target_actuation):
        first_loop = True
        state_i = nav_state.copy()
        k = np.array([0, 0, 0, 0])

        while state_i[3] > 0:
            # RK4 integration
            if first_loop:
                k1 = self.get_nav_state_derivative(state_i, a_state, drag_state)
                k2 = self.get_nav_state_derivative(state_i + 0.5*g.dt_guidance*k1, a_state, drag_state)
                k3 = self.get_nav_state_derivative(state_i + 0.5*g.dt_guidance*k2, a_state, drag_state)
                k4 = self.get_nav_state_derivative(state_i + g.dt_guidance*k3, a_state, drag_state)
                k = 0.1666*(k1 + 2.0*k2 + 2.0*k3 + k4)
                state_i = state_i + g.dt_guidance*k
                first_loop = False
            else:
                k1 = self.get_nav_state_derivative(state_i, g.target_actuation, drag_state)
                k2 = self.get_nav_state_derivative(state_i + 0.5*g.dt_guidance*k1, g.target_actuation, drag_state)
                k3 = self.get_nav_state_derivative(state_i + 0.5*g.dt_guidance*k2, g.target_actuation, drag_state)
                k4 = self.get_nav_state_derivative(state_i + g.dt_guidance*k3, g.target_actuation, drag_state)
                k = 0.1666*(k1 + 2.0*k2 + 2.0*k3 + k4)
                state_i = state_i + g.dt_guidance*k
        
        t_post = state_i[3] / k[3]

        return state_i[2] - k[2] * t_post
    
    def get_state(self, nav_state, drag_state, a_state):
        return np.array([
            self.get_z_apogee(nav_state, drag_state, a_state),
            a_state
        ])
    
    def get_state_derivative(self, state, input, nav_state, drag_state):
        return np.array([
            (self.get_z_apogee(nav_state, drag_state, state[1]) - state[0]) / g.dt_guidance,
            self.get_actuator_derivative(input, state[1])
        ])
    
    def get_linearized(self, state_0, input_0, nav_state_0, drag_state_0, e=1e-6):
        #forward finite difference approx        
        state_derivative_0 = self.get_state_derivative(state_0, input_0, nav_state_0, drag_state_0)

        J_state = np.zeros((2, 2))
        J_input = np.zeros((1, 2))

        for i in range(2):
            state_i = state_0.copy()
            state_i[i] += e

            state_partial_i = (self.get_state_derivative(state_i, input_0, nav_state_0, drag_state_0) - state_derivative_0) / e
            J_state[i] = state_partial_i
        
        J_input[0] = (self.get_state_derivative(state_0, input_0 + e, nav_state_0, drag_state_0) - state_derivative_0) / e
        
        return J_state, J_input
    
    def get_drag_output(self, drag_state, drag_input):
        nav_derivative = self.get_nav_state_derivative(drag_input[0:4], drag_input[4], drag_state)
        return np.array([nav_derivative[1], nav_derivative[3]])
    
    def get_linearized_drag_output(self, drag_state_0, drag_input_0, e=1e-6):
        #forward finite difference approx        
        output_0 = self.get_drag_output(drag_state_0, drag_input_0)

        J_state = np.zeros((4, 2))
        J_input = np.zeros((5, 2))

        for i in range(4):
            drag_state_i = drag_state_0.copy()
            drag_state_i[i] += e

            state_partial_i = (self.get_drag_output(drag_state_i, drag_input_0) - output_0) / e
            J_state[i] = state_partial_i
        
        for i in range(5):
            drag_input_i = drag_input_0.copy()
            drag_input_i[i] += e

            input_partial_i = (self.get_drag_output(drag_state_0, drag_input_i) - output_0) / e
            J_input[i] = input_partial_i
        
        return J_state, J_input
    
    def get_kalman_A(self, dt):
        #regular matrix, assumes constant accel
        return np.array([
            [1, dt, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, dt],
            [0, 0, 0, 1]
        ])
    
    def get_kalman_B(self, dt):
        #input augmented with accelerometer readings
        return np.array([
            [0.5*dt**2, 0],
            [dt, 0],
            [0, 0.5*dt**2],
            [0, dt]
        ])
    
    def get_drag_A(self):
        return np.identity(4)
    
    def get_drag_B(self):
        return np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0]
        ])