#!/bin/env bash

# Builds monolith for simulator 

set -e

nice -10 make VEXIDE_SIMULATION=1 quick -j $(($(nproc) - 2)) 