# Message with constraints per stage in the horizon

# Constraints have the following structure: [F_{0-V,0,0|N} ... F_{0-V,C,0|N} ... F_{0-V,0-C,N|N}], with:
# - number of values per constraint V
# - number of constraints C
# - number of stages N

# Header
Header header

# MPC header
mpc_msgs/MpcHeader mpc_header

# Array with length V*C*(N+1)
float64[] con

# Array with amount of relevant constraints per stage
int8[] n_relevant_con
