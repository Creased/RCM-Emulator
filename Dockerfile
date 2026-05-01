FROM ubuntu:24.04

# Build environment for RCM Payload Emulator
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    libunicorn-dev \
    libsdl2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work

# Build command: docker run --rm -v $(pwd):/work rcm_emu make
CMD ["make"]
