# Message with a trajectory of inputs and states over the horizon

# NOTE:
# - This could be a predicted trajectory, reference trajectory, etc.

# Header
Header header

# MPC header
mpc_msgs/MpcHeader mpc_header

# Names
string[] u_names
string[] x_names

# Trajectory with length (len(u_names)+len(x_names))*(N+1)
float64[] traj
