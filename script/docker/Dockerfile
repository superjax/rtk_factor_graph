FROM ubuntu:20.04 as base
MAINTAINER James Jackson(superjax08 @gmail.com)

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="America/Los_Angeles"

#Set up development environment
RUN apt-get update && \
    DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    build-essential \
    clang-tools \
    git \
    cmake \
    lcov \
    ccache \
    libboost-system-dev \
    libatlas-base-dev && rm -rf /var/lib/apt/lists/*
