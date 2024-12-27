#!/bin/env bash

# Builds and runs program
# Then extracts timing results from program logs to append them to a log file

./scripts/build-sim.sh &&
  unbuffer cargo v5 run --file bin/sim/monolith.bin --slot 7 |
  tee >(
    tee pregrep.log |
      grep --text --line-buffered '^\[PF.END\]' |
      mawk -Winteractive '{printf "[%s]\t%s ->\n\t%s\n", strftime("%F %T"), "'"$1"'", $0}' >>log
  )
