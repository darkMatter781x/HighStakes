#!/usr/bin/env /usr/bin/bash
# To run this script, you need to have the following installed:
# qemu
# rust
# git-bash (for windows)
# pros toolchain

# run with ./script.ps1 on windows

start_dir="$(pwd)"

if ! command cargo --version &>/dev/null; then
    echo "The rust toolchain must be installed in order to build the simulator. Please install rust and try again."
    exit 1
fi

if [ ! -d vex-v5-qemu ]; then
    echo "Attempting to install the vex-v5-qemu kernel and simulator"
    if ! command git --version &>/dev/null; then
        echo "Git must be installed in order to install dependencies. Please install git and try again."
        exit 1
    fi
    git clone https://github.com/vexide/vex-v5-qemu.git vex-v5-qemu
    cd vex-v5-qemu
    if cargo --version | grep -q "nightly"; then
        # We require nightly to build the vex v5 qemu kernel
        rustup override set nightly --path .
        exit 1
    fi
    cd kernel
    cargo build
    cd ../simulator
    cargo build
    cd "$start_dir"
fi

PATH="/usr/bin:$PATH:/c/Program Files/qemu:$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin"

if ! command qemu-system-arm --version &>/dev/null; then
    echo "An 8.x.x version of qemu must be installed. Please install qemu and try again.
    I used the following link, found at https://qemu.weilnetz.de/w64/2023/ , to install it:
    https://qemu.weilnetz.de/w64/2023/qemu-w64-setup-20231224.exe"
    exit 1
fi

if ! qemu-system-arm --version | grep "v8" ; then
    echo "Qemu is installed but is not the correct version"
    echo "An 8.x.x version of qemu must be installed. Please install qemu and try again.
    I used the following link, found at https://qemu.weilnetz.de/w64/2023/ , to install it:
    https://qemu.weilnetz.de/w64/2023/qemu-w64-setup-20231224.exe"
    exit 1
fi

if ! command arm-none-eabi-addr2line.exe --version &>/dev/null; then
    echo "the pros toolchain must be installed. Please install the pros toolchain and try again.
    (AKA the arm-none-eabi toolchain)"
    exit 1
fi

if ! command make --version &>/dev/null; then
    echo "Make must be installed. Please install make and try again. 
    (This is usually included with the pros toolchain)"
    exit 1
fi

# make the binary
make VEXIDE_SIMULATION=1 quick -j

# run the binary
cd vex-v5-qemu/simulator
cargo run -- ../../bin/monolith.bin