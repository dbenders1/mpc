#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <number_of_files>"
    exit 1
fi

# Concatenate timing results in mpc_profiler_i.json for every ith MPC layer into one file mpc_profiler.json
# Input ($1): number of hierarchical layers
cp mpc_profiler_0.json temp0.json
for ((i = 1; i < $1; i++)); do
    jq --slurpfile json_arr mpc_profiler_$i.json '.traceEvents += $json_arr[0].traceEvents' temp$((i - 1)).json >temp$i.json
    rm temp$((i - 1)).json
done
mv temp$((i - 1)).json mpc_profiler.json
echo "Concatenated mpc_profiler_i.json for i = 0, ..., $((i - 1)) into mpc_profiler.json"
