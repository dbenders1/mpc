# Message with interpolation data

# NOTE:
# - This message contains the RK4 predicted state and the interpolated states between stage 0 and 1 used for the feedback law

# Header
Header header

# MPC header
mpc_msgs/MpcHeader mpc_header

# Array with RK4 and interpolated states
float64[] rk4_states
float64[] interp_states
