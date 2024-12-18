import casadi
import numpy as np
from include import helpers

helpers.load_forces_path()

import forcespro.nlp


# Custom RK4 implementation
def solve_rk4(continuous_model, x, u, param, settings):
    dt = settings.integrator_options['stepsize']
    n_steps = settings.integrator_options['steps']
    if settings.integrator_options['use_dt_feedback_law']:
        v = u - settings.offline_comp.get_K_delta() @ x
    for i in range(n_steps):
        if settings.integrator_options['use_dt_feedback_law']:
            u = v + settings.offline_comp.get_K_delta() @ x
        x = rk4_step(continuous_model, x, u, param, settings, dt)
    return u, x

def rk4_step(continuous_model, x, u, param, settings, dt):
    k1 = continuous_model(x, u, param, settings)
    k2 = continuous_model(x + dt / 2 * k1, u, param, settings)
    k3 = continuous_model(x + dt / 2 * k2, u, param, settings)
    k4 = continuous_model(x + dt * k3, u, param, settings)
    return x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def add_dual(model, nh):
    model.nu += nh
    model.continuous_model = lambda x, u, param, settings: casadi.vstack(model.continuous_model(x, u, param, settings),
                                                                         np.zeros((nh, 1)))  # lambdas have no dynamics
    

# Dynamics, i.e. equality constraints #
# This class contains models to choose from
# They can be coupled with physical limits using Systems defined in systems.py
# See Bicycle model for an example of the required parameters
class DynamicModel:

    def __init__(self, system):
        self.nvar = self.nu + self.nx
        self.system = system
        self.control_inputs = dict()
        self.possible_inputs_to_vehicle = []

    def __str__(self):
        result = 'Dynamical Model: ' + str(type(self)) + '\n' +\
               'System: ' + str(self.system) + '\n'

        if hasattr(self, 'interfaces'):
            result += 'Interfaces: '

            for interface in self.interfaces:
                result += interface + " "

            result += "\n"

        result += 'States: ' + str(self.states) + '\n'
        result += 'Inputs: ' + str(self.inputs) + '\n'
        return result

    # Appends upper bounds from system
    def upper_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.upper_bound[input])

        for state in self.states:
            result = np.append(result, self.system.upper_bound[state])

        return result

    # Appends lower bounds from system
    def lower_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.lower_bound[input])

        for state in self.states:
            result = np.append(result, self.system.lower_bound[state])

        return result

    def discretize_dynamics(self, z, param, settings):
        """
        @param z: state vector (u, x)
        @param param: Runtime parameters
        @param settings: System settings, including integrator stepsize in seconds
        @return:
        """
        x = z[self.nu:self.nu+self.nx]
        u = z[0:self.nu]

        # We use an explicit RK4 integrator here to discretize continuous dynamics
        result = forcespro.nlp.integrate(
            self.continuous_model,
            x,
            u,
            param,
            settings,
            integrator = forcespro.nlp.integrators.RK4,
            stepsize = settings.integrator_options['steps'] * settings.integrator_options['stepsize'],
            steps = settings.integrator_options['steps'])

        return result

    def get_state(self, z, state_name, required=False):
        if state_name not in self.states:
            assert not required, "Required state {} was not part of the model".format(state_name)
            return 0.
        return z[self.nu + self.states.index(state_name)]

    def get_input(self, z, input_name, required=False):
        if input_name not in self.inputs:
            assert not required, "Required input {} was not part of the model".format(input_name)
            return 0.
        return z[self.inputs.index(input_name)]

    def get_z_index(self, z, name, required=False):
        if (name not in self.inputs) and (name not in self.states):
            assert not required, "Required variable {} was not part of the model".format(name)
            return 0.
        if name in self.inputs:
            return z[self.inputs.index(name)]
        else:
            return z[self.nu + self.states.index(name)]

    def get_idx(self, name, required=False):
        if (name not in self.inputs) and (name not in self.states):
            # print("Unknown variable {} was indexed! Setting to zero".format(name))
            assert not required, "Required variable {} was not part of the model".format(name)
            return 0.
        if name in self.inputs:
            return self.inputs.index(name)
        else:
            return self.nu + self.states.index(name)


class DroneModel(DynamicModel):

    def __init__(self, system, options, offline_comp=None):
        # Check whether model should be nonlinear and whether yaw component should be included or not
        self.nonlin = options['nonlin']
        self.with_yaw = options['with_yaw']
        # Physical notion is used to generate state and input reference from higher-layer MPCC to lower-layer RMPC
        self.physical_only = options['physical_only']
        self.use_input_rates = options['use_input_rates']
        self.use_slack = options['use_slack']
        self.use_ct_feedback_law = options['use_ct_feedback_law']

        # Physical states include all system states, except virtual states used for path following
        # They should always come first in the states list!
        if self.use_input_rates:
            self.states = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'thrust', 'phi_c', 'theta_c', 'psi_c']
            self.states_from_sensor = [True, True, True, True, True, True, True, True, True, True, True, True, False, False,False, False, False, False]
            self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, True, True, True, True, True, True, False, False,False, False, False, False]
            self.inputs = ['delta_phi_c', 'delta_theta_c', 'delta_psi_c', 'thrust_c']
            self.possible_inputs_to_vehicle = self.inputs
        else:
            self.states = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'thrust']
            self.states_from_sensor = [True, True, True, True, True, True, True, True, True, False, False,]
            self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, True, True, True, False, False,]
            self.inputs = ['phi_c', 'theta_c', 'psi_c', 'thrust_c']
            self.possible_inputs_to_vehicle = self.inputs

        if self.use_slack:
            self.inputs.append('slack')

        if self.use_ct_feedback_law:
            self.K_delta = offline_comp.get_K_delta()

        self.nu = len(self.inputs)
        self.nu_physical = len(self.inputs)
        self.nx = len(self.states)
        self.nx_physical = len(self.states)
        super(DroneModel, self).__init__(system)

    def discretize_dynamics(self, z, param, settings):

        # When using input rates in planner: adjust inputs and states for discretization with feedback law
        u = [None] * self.nu
        u[0] = settings.model.get_z_index(z, 'phi_c', True)
        u[1] = settings.model.get_z_index(z, 'theta_c', True)
        u[2] = settings.model.get_z_index(z, 'psi_c', True)
        u[3] = settings.model.get_z_index(z, 'thrust_c', True)
        x = z[self.nu:self.nu+self.nx]
        x = list(x[i] for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

        if self.use_input_rates:
            # Desired angle rates around body frames [rad]
            delta_phi = settings.model.get_z_index(z, 'delta_phi_c', True)
            delta_theta = settings.model.get_z_index(z, 'delta_theta_c', True)
            delta_psi = settings.model.get_z_index(z, 'delta_psi_c', True)
            u[0] = u[0] + delta_phi
            u[1] = u[1] + delta_theta
            u[2] = u[2] + delta_psi

        result_u, result_x = solve_rk4(self.continuous_model, x, u, param, settings)

        if self.use_input_rates:
            #return np.array([result_x[0:9], result_u[0:3]])
            return np.array([result_x[0:10], result_u[0], result_u[1], result_u[2]])
        else:
            return result_x

    def continuous_model(self, x, u, param, settings):
        # if self.use_ct_feedback_law:
        #     u = u + self.K_delta @ x

         
        vx = x[3]
        vy = x[4]
        vz = x[5]
        phi = x[6]
        theta = x[7]
        psi = x[8]
        thrust = x[9]

        # Inputs
        phi_c = u[0]
        theta_c = u[1]
        psi_c = u[2]
        thrust_c = u[3]

        # Identified constants
        # Roll and pitch
        A = -5.55
        B = 5.55

        # Yaw
        A_yaw = -1.773
        B_yaw = 1.773

        # Thrust
        A_average = -20
        B_average = 20

        # Constants
        g = 9.81

        # Calculate derivatives
        dx = vx
        dy = vy
        dz = vz
        if not self.nonlin:
            dvx = g * theta
            dvy = -g * phi
            dvz = thrust
        else:
            dvx = thrust * (casadi.sin(phi) * casadi.sin(psi) + casadi.cos(phi) * casadi.sin(theta) * casadi.cos(psi))
            dvy = thrust * (-casadi.sin(phi) * casadi.cos(psi) + casadi.cos(phi) * casadi.sin(theta) * casadi.sin(psi))
            dvz = thrust * (casadi.cos(phi) * casadi.cos(theta)) - g
        
        dphi = A * phi + B * phi_c
        dtheta = A * theta + B * theta_c
        dpsi =  A_yaw * psi + B_yaw * psi_c
        dthrust = A_average * thrust + B_average * thrust_c

        return np.array([dx, dy, dz, dvx, dvy, dvz, dphi, dtheta, dpsi, dthrust])
