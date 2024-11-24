#!/usr/bin/env bash

# This script is used to mock the to switch the cwd before calling gdb.
# This is because the vscode c++ extension will run miDebuggerPath from
# whatever directory the file is in.
#   Note: This only occurs if miDebuggerPath recognized as a path to an excutable,
#         otherwise it seemingly runs from vscode's cwd.

gdb_exec="/usr/bin/gdb-multiarch"
workspace="/workspaces/HighStakes"
log_path="./gdb/logs"
gdb_args=$@

if ! [ -d "$workspace" ]; then
    echo "Error: Workspace directory does not exist: $workspace" >&2
    exit
fi

run_gdb() {
    $gdb_exec --cd="$workspace" $gdb_args 
}

if ! [ -z "$log_path"  ]; then
    mkdir -p "$log_path"
    set > "$log_path/env.log"
    echo "$gdb_args" > "$log_path/args.log"
    tee "$log_path/to-gdb.log" | run_gdb | tee "$log_path/to-vsc.log"
else
   run_gdb
fi
