#!/bin/env bash

# Builds monolith for simulator 

set -e

nice -10 make SIM=1 -j $(($(nproc) - 2)) ${@:-quick}