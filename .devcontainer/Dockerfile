FROM rust:1.67 as builder

RUN git clone https://github.com/vexide/vex-v5-qemu.git /vex-v5-qemu

WORKDIR /vex-v5-qemu
RUN cd packages/kernel; cargo build --target-dir /target/kernel

RUN cd packages/client-cli; cargo install --path . --root /target/client-cli
RUN ls -R /target

RUN find /target -iname "client-cli"
RUN find /target -iname "kernel"

FROM mcr.microsoft.com/devcontainers/cpp:1.0-jammy

RUN sudo apt-get update
RUN sudo apt-get -y install qemu-system python3 gdb-multiarch

# add pros toolchain to path 
ENV PATH="$PATH:/home/vscode/.vscode-remote/data/User/globalStorage/sigbots.pros/install/pros-toolchain-linux/bin" 

COPY --from=builder /target/client-cli/bin/client-cli /usr/local/bin/simulator

ENV V5_SIM_KERNEL_PATH="$HOME/.vex-v5-qemu/kernel"
COPY --from=builder /target/kernel/armv7a-none-eabi/debug/kernel $V5_SIM_KERNEL_PATH