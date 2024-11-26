import os

import numpy as np
import scipy.io


class TmpcOffline:
    def __init__(self) -> None:
        filename = "offline_comps_tracking.mat"
        self.mat = scipy.io.loadmat(
            os.path.dirname(os.path.realpath(__file__)) + "/mpc-sdp/" + filename, squeeze_me=True
        )
        self.Q = np.array(self.mat["Q"])
        self.R = np.array(self.mat["R"])
        self.__P_delta = np.array(self.mat["P"])
        self.__K_delta = np.array(self.mat["K"])
        self.__alpha = self.mat["alpha"]
        self.__c_s = np.array(self.mat['c_s'])
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
