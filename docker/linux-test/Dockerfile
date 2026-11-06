# Linux build + test image for rflysim_macV1 (Debian 12). Used by scripts/linux_docker_test.sh.
# GUI stages run on a virtual X display (Xvfb) with Mesa software OpenGL (llvmpipe).
# BASE: an arch-specific digest builds a foreign-arch image without buildx (see scripts/package.sh --linux).
ARG BASE=debian:bookworm-slim
FROM ${BASE}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl git build-essential pkg-config cmake ninja-build file \
      libudev-dev libasound2-dev \
      libx11-6 libxi6 libxkbcommon0 libxcursor1 libxrandr2 libgl1 libegl1 libgl1-mesa-dri libglib2.0-0 \
      xvfb xauth \
      python3 python3-venv \
      procps iproute2 lsof binutils desktop-file-utils rsync sudo apt-utils gnupg \
    && rm -rf /var/lib/apt/lists/*
# PX4 builds from a source tree copied in by the test script; let git read it.
RUN git config --system --add safe.directory '*'
# No sound card in a container: send ALSA's default device to a null sink.
RUN printf 'pcm.!default { type null }\nctl.!default { type hw card 0 }\n' >/etc/asound.conf
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable --no-modify-path \
    && chmod -R a+rwX /opt/cargo /opt/rustup
WORKDIR /work
