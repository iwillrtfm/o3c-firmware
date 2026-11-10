# Build environment for the O3C firmware, the o3c host tool and wchisp. Used by ./build;
# nothing of it is installed on the host.
FROM debian:trixie
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
      ca-certificates curl git build-essential pkg-config python3 \
      libudev-dev libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable
COPY firmware/rust-toolchain.toml /tmp/rust-toolchain.toml
RUN rustup toolchain install "$(sed -n 's/^channel = "\(.*\)"/\1/p' /tmp/rust-toolchain.toml)" --profile minimal \
      --component llvm-tools --target riscv32imac-unknown-none-elf
RUN cargo install wchisp --version 0.3.0 --locked --root /opt/tools && rm -rf /opt/cargo/registry
