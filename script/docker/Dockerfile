FROM ubuntu:18.04 as base
MAINTAINER James Jackson (superjax08@gmail.com)

# Set up development environment
RUN apt-get update && apt-get install -y \
    build-essential \
    clang-tools \
    git \
    zlib1g-dev \
    libcurl4-openssl-dev \
    lcov \
    ccache \
    libboost-system-dev \
    && rm -rf /var/lib/apt/lists/*

# Build the newest CMake
RUN git clone https://github.com/Kitware/CMake && cd CMake && ./bootstrap --system-curl --parallel=16 && make -j16 && make install
