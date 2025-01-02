#!/bin/env bash

# Builds and runs program
# Then extracts timing results from program logs to append them to a log file

change_description="$1"
log_file="${2:-"perf-testing.log"}"

reader() {
  mawk_script='
/^\[ProfSummary.Start\]/{f=1; printf "%s ->\n", change; next}; 
/^\[ProfSummary.End\]/{f=0};
{ if (f) printf "  [%s]\t%s\n", strftime("%F %T"), $0}'

  sed -u "s%\x1B\[[0-9;]*[a-zA-Z]%%g" |
    stdbuf -o0 tr -d '\000' |
    mawk -Winteractive -v change="$change_description" "$mawk_script" >>"$log_file"
}

./scripts/build-sim.sh &&
  unbuffer v5 run --file bin/sim/monolith.bin --slot 8 |
  tee >(reader)
