FROM rust:1.83-bullseye as rust-builder

# ------------
# Build simulator
# ------------   
FROM rust-builder as simulator

RUN mkdir /vex-v5-qemu                                                                                \
  && curl -L --output                                                                                 \
    -H "Accept: application/vnd.github+json"                                                          \
    -H "X-GitHub-Api-Version: 2022-11-28"                                                             \
    https://api.github.com/repos/vexide/vex-v5-qemu/tarball/f9a57b3516fdda27c5c9fc4d02128882e45e3ee9  \
  | tar zxf - -C /vex-v5-qemu --strip-components=1

WORKDIR /vex-v5-qemu
RUN cd packages/kernel; cargo build --target-dir /target/kernel

RUN cd packages/client-cli; cargo install --path . --root /target/client-cli

# ------------
# Build cargo-v5
# ------------   
FROM rust-builder as cargo-v5
RUN apt-get update
RUN apt-get install libudev-dev -y
RUN apt-get clean # Cleanup image

RUN cargo install                                 \
  --git https://github.com/meiszwflz/cargo-v5.git    \
  --rev a849dd5937f78edca2b28e0978aecaa2f041317b  \
  --no-default-features                           \
  -F "clap"                                       \
  -F "field-control"                              \
  --root /target/cargo-v5

#####################################
#                                   #
#     Build Devcontainer Image      #
#                                   #
#####################################
FROM mcr.microsoft.com/devcontainers/cpp:1-noble
# ------------
# Install Required Packages
# ------------   
RUN sudo apt-get update
COPY ./.devcontainer/packagelist /packagelist
RUN sudo apt-get -y install $(cat /packagelist | sed 's/\r//') 
RUN rm /packagelist # Cleanup image
RUN sudo apt-get clean # Cleanup image

# ------------
# Install PIP packages
# ------------
COPY ./.devcontainer/pip_packagelist /pip_packagelist
RUN pip install --break-system-packages $(cat /pip_packagelist | sed 's/\r//') 
RUN rm /pip_packagelist # Cleanup image

# ------------
# Install Clangd
# ------------
RUN curl -sLo clangd.zip $(                                                                 \
curl -s https://api.github.com/repos/clangd/clangd/releases/latest                      \
| jq -r '[.assets[] | select(.name | test ("^clangd-linux"))][0].browser_download_url'  \
)                                                                                         \
&& unzip clangd.zip -d /usr/local/share                                                   \
&& mv /usr/local/share/clangd_*/ /usr/local/share/clangd                                  \
&& rm clangd.zip

ENV PATH="$PATH:/usr/local/share/clangd/bin"

# ------------ 
# Install ARM Toolchain
# ------------
COPY --from=ghcr.io/lemlib/pros-build:v3.0.0 /arm-none-eabi-toolchain /usr/local/share/arm-none-eabi
ENV PATH="$PATH:/usr/local/share/arm-none-eabi/bin"  

# Copy the simulator binary
COPY --from=simulator /target/client-cli/bin/client-cli /usr/local/bin/simulator
# Copy the cargo-v5 binary
COPY --from=cargo-v5 /target/cargo-v5/bin/cargo-v5 /usr/local/bin/cargo-v5
COPY --chmod=777 <<EOF /usr/local/bin/v5
#!/bin/env bash
cargo-v5 v5 \$@
EOF

# Clone template sources so we can reference it when debugging
COPY ./project.pros ./project.map.json ./.devcontainer/template_src.bash /
ENV TEMPLATES_SOURCE_PATH="$HOME/.templates/"
RUN /template_src.bash /project.pros /project.map.json "$TEMPLATES_SOURCE_PATH"
RUN rm /project.pros /project.map.json /template_src.bash # Cleanup image

# Copy the sim kernel, as its required to run the simulator
ENV V5_SIM_PATH="$HOME/.vex-v5-qemu"
ENV V5_SIM_KERNEL_PATH="$V5_SIM_PATH/kernel"
COPY --from=simulator /target/kernel/armv7a-none-eabi/debug/kernel $V5_SIM_KERNEL_PATH

# Copy the sim source, so we can reference it when debugging
ENV V5_SIM_SOURCE_PATH="$V5_SIM_PATH/source"
COPY --from=simulator /vex-v5-qemu $V5_SIM_SOURCE_PATH

# Enable running of .gdbinit
RUN mkdir -p /home/vscode/.config/gdb && echo "add-auto-load-safe-path /workspaces/HighStakes/.gdbinit" >/home/vscode/.config/gdb/gdbinit

RUN sudo usermod -aG dialout vscode