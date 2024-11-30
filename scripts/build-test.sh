#!/bin/env bash

# Builds unit tests and runs them

set -e

cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cmake --build build
cd build
ctest --output-on-failure