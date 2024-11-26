# Message with number of MPC layer to start

# NOTE:
# - layer = 0 indicates the lowest-layer MPC
# - con is ordered as constraint 0 A[0], A[1], b, constraint 1 A[0], A[1], b, etc.

# Header
Header header

# Control loop count of TMPC that initiated the PMPC (for synchronization and checking reference validity)
int64 count

# Reset indication for restarting MPC from current position
bool reset

# The current position
float64[] position

# The current obstacle avoidance constraints
float64[] constraints
int8 n_relevant_constraints
