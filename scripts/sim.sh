#!/bin/env bash

# Runs simulator and sets up pipes listening to the simulator's output

echo \"starting simulator!\"

tmpdir="$(mktemp -d)"
pipe="$tmpdir/pipe"
mkfifo "$pipe"

# ensures that tmp files are deleted
cleanup() {
  echo "Cleaning up: $tmpdir"
  rm -Rf "$tmpdir"
}

# if prompted to exit early, make sure to cleanup
trap "cleanup; exit 3" SIGINT SIGTERM

# Start simulator in the background
simulator $@ | tee "$pipe" &

# Start listener
while true; do
    if read line <"$pipe"; then
        echo "simulator: $line" >log
    fi
done

# Cleanup
rm -rf "$tmpdir"