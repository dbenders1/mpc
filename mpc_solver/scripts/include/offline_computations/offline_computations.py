import ctypes
import json
import math
import os

import casadi as ca
import numpy as np
import scipy.io


class RohmpcOffline:
    def __init__(self) -> None:
        # Load .mat file
        filename = "offline_design.mat"
        mat = scipy.io.loadmat(
            os.path.dirname(os.path.realpath(__file__)) + "/mpc-sdp/" + filename,
            squeeze_me=True,
        )
        self.g = mat["g"]
        self.ts = mat["ts"]
        self.nx = mat["nx"]
        self.C = mat["C"]
        self.mass = mat["mass"]
        self.inertia = mat["inertia"]
        self.B_allocation = mat["B_allocation"]
        self.motor_tau = mat["motor_tau"]
        self.thrust_map = mat["thrust_map"]
        self.kd = mat["kd"]
        self.E = mat["E"]
        self.w_bias = mat["w_bias"].reshape(-1, 1)
        self.p_min = mat["p_min"]
        self.p_max = mat["p_max"]
        self.att_min = mat["att_min"]
        self.att_max = mat["att_max"]
        self.v_min = mat["v_min"]
        self.v_max = mat["v_max"]
        self.wb_min = mat["wb_min"]
        self.wb_max = mat["wb_max"]
        self.t_min = mat["t_min"]
        self.t_max = mat["t_max"]
        self.t_hover = mat["t_hover"]
        self.Q = mat["Q"]
        self.R = mat["R"]
        self.P = mat["P"]
        self.c_s = mat["c_s"]
        self.c_o = mat["c_o"]
        self.L = mat["L"]

        # Compute X and Y
        p_xidc = mat["p_xidc"]
        x_sym = ca.SX.sym("x", self.nx, 1)
        X_arr = mat["X_arr"]
        if len(X_arr.shape) < 3:
            X = X_arr
        else:
            nX = X_arr.shape[2]
            p = x_sym[p_xidc]
            X = X_arr[:, :, 0]
            for i in range(1, nX):
                X = X + X_arr[:, :, i] * p[i - 1]
        Y_arr = mat["Y_arr"]
        if len(Y_arr.shape) < 3:
            Y = Y_arr
        else:
            nY = Y_arr.shape[2]
            p = x_sym[p_xidc]
            Y = Y_arr[:, :, 0]
            for i in range(1, nY):
                Y = Y + Y_arr[:, :, i] * p[i - 1]

        # Create P_delta matrix (no function support yet)
        self.P_delta = ca.inv(X)

        # Create K_delta matrix (no function support yet)
        self.K_delta = Y @ self.P_delta

        # Load empirical tightening constants
        tightening_path = (
            os.path.dirname(os.path.realpath(__file__))
            + "/../../../../../mpc_model_id_mismatch/data/tightening_results/tightening.json"
        )
        with open(tightening_path, "r") as file:
            tightening_data = json.load(file)
        self.epsilon = tightening_data["epsilon"]
        self.rho_c = tightening_data["rho_c"]
        self.w_bar_c = tightening_data["w_bar_c"]

        # Compute RPI bounds for controller error and total error
        self.s_rpi = self.w_bar_c / self.rho_c
        self.alpha = self.s_rpi + self.epsilon

    def get_alpha(self):
        return self.alpha

    def get_B_allocation(self):
        return self.B_allocation

    def get_C(self):
        return self.C

    def get_constraint_quantities_t(self):
        return [
            self.p_min,
            self.p_max,
            self.att_min,
            self.att_max,
            self.v_min,
            self.v_max,
            self.wb_min,
            self.wb_max,
            self.t_min,
            self.t_max,
        ]

    def get_c_o(self):
        return self.c_o

    def get_c_s(self):
        return self.c_s

    def get_E(self):
        return self.E

    def get_epsilon(self):
        return self.epsilon

    def get_g(self):
        return self.g

    def get_inertia(self):
        return self.inertia

    def get_kd(self):
        return self.kd

    def get_K_delta(self):
        return self.K_delta

    def get_L(self):
        return self.L

    def get_mass(self):
        return self.mass

    def get_model_quantities_t(self):
        return [
            self.g,
            self.mass,
            self.inertia,
            self.B_allocation,
            self.thrust_map,
            self.kd,
            self.E,
            self.w_bias,
        ]

    def get_P(self):
        return self.P

    def get_P_delta(self):
        return self.P_delta

    def get_Q(self):
        return self.Q

    def get_R(self):
        return self.R

    def get_rho_c(self):
        return self.rho_c

    def get_s_array(self, ts, n, growing=True):
        if growing:
            s = np.zeros((n, 1))
            for k in range(n):
                s[k] = self.get_s_k(ts, k)
        else:
            s = self.s_rpi * np.ones((n, 1))
        return s

    def get_s_k(self, ts, k):
        return self.s_rpi * (1 - math.exp(-self.rho_c * k * ts))

    def get_s_rpi(self):
        return self.s_rpi

    def get_thrust_map(self):
        return self.thrust_map

    def get_t_hover(self):
        return self.t_hover

    def get_w_bar_c(self):
        return self.w_bar_c

    def get_w_bias(self):
        return self.w_bias


class TmpcOffline:
    def __init__(self) -> None:
        filename = "offline_comps_tracking.mat"
        self.mat = scipy.io.loadmat(
            os.path.dirname(os.path.realpath(__file__)) + "/mpc-sdp/" + filename,
            squeeze_me=True,
        )
        self.Q = np.array(self.mat["Q"])
        self.R = np.array(self.mat["R"])
        self.__P_delta = np.array(self.mat["P"])
        self.__K_delta = np.array(self.mat["K"])
        self.__alpha = self.mat["alpha"]
        self.__c_s = np.array(self.mat["c_s"])
        self.__c_s = np.repeat(self.__c_s, 2)
        self.__c_o = self.mat["c_o"]

    def get_Q(self):
        return self.Q

    def get_R(self):
        return self.R

    def get_P_delta(self):
        return self.__P_delta

    def get_K_delta(self):
        return self.__K_delta

    def get_alpha(self):
        return self.__alpha

    def get_c_s(self):
        return self.__c_s

    def get_c_o(self):
        return self.__c_o

    def load_offline_comps(self, z, param):
        # Update offline design parameters during runtime here if necessary
        pass
