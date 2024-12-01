#!/bin/env bash

# Builds monolith for simulator 

set -e

make VEXIDE_SIMULATION=1 quick -j $(nproc) 