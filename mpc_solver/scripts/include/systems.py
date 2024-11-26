import numpy as np


class Hovergames:

    def __str__(self):
        return self.name

    def __init__(self, use_input_rates):
        self.name = 'Hovergames'
        self.lower_bound = dict()
        self.upper_bound = dict()

        self.use_input_rates = use_input_rates

        # Maximum roll and pitch angle
        max_angle = 30  # deg
        max_angle_rate = 60  # deg/s

        self.lower_bound['x'] = -15.0
        self.upper_bound['x'] = 15.0

        self.lower_bound['y'] = -15.0
        self.upper_bound['y'] = 15.0

        self.lower_bound['z'] = 0.0
        self.upper_bound['z'] = 4.0

        self.lower_bound['vx'] = -2.0
        self.upper_bound['vx'] = 2.0

        self.lower_bound['vy'] = -2.0
        self.upper_bound['vy'] = 2.0

        self.lower_bound['vz'] = -2.0
        self.upper_bound['vz'] = 2.0

        self.lower_bound['phi'] = -np.pi / 180 * max_angle
        self.upper_bound['phi'] = np.pi / 180 * max_angle

        self.lower_bound['theta'] = -np.pi / 180 * max_angle
        self.upper_bound['theta'] = np.pi / 180 * max_angle

        self.lower_bound['psi'] = -np.pi / 180 * max_angle
        self.upper_bound['psi'] = np.pi / 180 * max_angle

        self.lower_bound['phi_c'] = -np.pi / 180 * max_angle
        self.upper_bound['phi_c'] = np.pi / 180 * max_angle

        self.lower_bound['theta_c'] = -np.pi / 180 * max_angle
        self.upper_bound['theta_c'] = np.pi / 180 * max_angle

        self.lower_bound['psi_c'] = -np.pi / 180 * max_angle
        self.upper_bound['psi_c'] = np.pi / 180 * max_angle

        if self.use_input_rates:
            self.lower_bound['delta_phi_c'] = -np.pi / 180 * max_angle_rate
            self.upper_bound['delta_phi_c'] = np.pi / 180 * max_angle_rate

            self.lower_bound['delta_theta_c'] = -np.pi / 180 * max_angle_rate
            self.upper_bound['delta_theta_c'] = np.pi / 180 * max_angle_rate

            self.lower_bound['delta_psi_c'] = -np.pi / 180 * max_angle_rate
            self.upper_bound['delta_psi_c'] = np.pi / 180 * max_angle_rate

        self.lower_bound['thrust'] = 5
        self.upper_bound['thrust'] = 15.0

        self.lower_bound['thrust_c'] = 5
        self.upper_bound['thrust_c'] = 15.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 5000
