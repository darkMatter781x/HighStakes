#!/bin/bash

# This script is used to generate the compile_commands.json file
# which is used by the clangd language server to provide intellisense.

set -e

make clean
pros build-compile-commands --compile-commands pros_compile_commands.json -- -j 8 &
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -S . -B build && cmake --build build &
wait
cat build/compile_commands.json pros_compile_commands.json | jq -s 'add' >compile_commands.json 
rm pros_compile_commands.json

echo "Updated compile_commands.json!"