# Message with reference trajectory and corresponding obstacle avoidance constraints

# All quantities are given in SI units

# Reference trajectory has the following structure: [u_{0-U,0|N} x_{0-X,0|N} u_{0-U,1|N} x_{0-X,1|N} ... u_{0-U,N|N} x_{0-X,N|N}], with:
# - number of inputs U
# - number of states X
# - number of stages N

# Reference obstacle avoidance constraints have the following structure: [F_{0-V,0,0|N} ... F_{0-V,C,0|N} ... F_{0-V,0-C,N|N}], with:
# - number of values per constraint V
# - number of constraints C
# - number of stages N

# Header
Header header

# Control loop count of TMPC that initiated the PMPC (for synchronization and checking reference validity)
int64 count

# Reference trajectory
float64[] ref_traj

# Obstacle avoidance constraints
float64[] ref_con
int8[] n_relevant_con
