FROM rust:1.67 as builder

RUN git clone https://github.com/meiszwflz/vex-v5-qemu.git --branch feat/sim/env-var-opts /vex-v5-qemu

WORKDIR /vex-v5-qemu
RUN rustup override set nightly
RUN rustup component add rust-src --toolchain nightly-x86_64-unknown-linux-gnu
RUN cd kernel; cargo build

RUN cd simulator; cargo install --path .

FROM mcr.microsoft.com/devcontainers/cpp:1.0-jammy

RUN sudo apt-get update
RUN sudo apt-get -y install qemu-system

COPY --from=builder /usr/local/cargo/bin/simulator /usr/local/bin/simulator

ENV V5_SIM_KERNEL=$HOME/.vex-v5-qemu/kernel 
COPY --from=builder /vex-v5-qemu/kernel/target/armv7a-none-eabi/debug/kernel $V5_SIM_KERNEL