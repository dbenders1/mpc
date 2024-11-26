#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <number_of_files> <process_mpc_profiler_feedback_law.json> <simple_sim_hovergames_profiler.json>"
    exit 1
fi

# Concatenate timing results in mpc_profiler_i.json for every ith MPC layer into one file mpc_profiler.json
# Input ($1): number of hierarchical layers
cp mpc_profiler_0.json temp0.json
for ((i = 1; i < $1; i++))
do
    jq --slurpfile json_arr mpc_profiler_$i.json '.traceEvents += $json_arr[0].traceEvents' temp$((i-1)).json > temp$i.json
    rm temp$((i-1)).json
done
echo "Processed mpc_profiler_i.json for i = 0, ..., $((i-1))"

# In case of feedback law (input $2 is true): add mpc_profiler_feedback_law.json as well
if [[ "$2" == "true" ]]; then
    jq --slurpfile json_arr mpc_profiler_feedback_law.json '.traceEvents += $json_arr[0].traceEvents' temp$((i-1)).json > mpc_profiler.json
    mv temp$((i-1)).json mpc_profiler_without_feedback_law.json
    echo "Processed mpc_profiler_feedback_law.json"
else
    echo "Not processing mpc_profiler_feedback_law.json"
    mv temp$((i-1)).json mpc_profiler.json
fi

# In case of simple_sim_hovergames (input $3 is true): add simple_sim_hovergames_profiler.json as well
if [[ "$3" == "true" ]]; then
    jq --slurpfile json_arr simple_sim_hovergames_profiler.json '.traceEvents += $json_arr[0].traceEvents' mpc_profiler.json > profiler.json
    echo "Processed simple_sim_hovergames_profiler.json"
else
    echo "Not processing simple_sim_hovergames_profiler.json"
fi
