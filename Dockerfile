FROM ubuntu:24.04

# Build environment for RCM Payload Emulator.
# git is needed so the container can bootstrap the Dear ImGui submodule
# (third_party/imgui) from the bind-mounted source tree on first build.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    git \
    libunicorn-dev \
    libsdl2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work

# Build command: docker run --rm -v $(pwd):/work rcm_emu
# (init the imgui submodule if it's missing, then build)
# The source is bind-mounted and owned by the host user, not root: without
# safe.directory, git refuses to touch it ("dubious ownership") and make never
# runs.
CMD ["sh", "-c", "git config --global --add safe.directory /work && git submodule update --init --recursive && make"]
