FROM ubuntu:18.04 as base
MAINTAINER James Jackson (superjax08@gmail.com)

# Set up development environment
RUN apt update
RUN apt install -y build-essential
RUN apt install -y clang-tools
RUN apt install -y git
RUN apt-get install -y zlib1g-dev
RUN apt-get install -y libcurl4-openssl-dev
RUN git clone https://github.com/Kitware/CMake && cd CMake && ./bootstrap --system-curl --parallel=16 && make -j16 && make install
RUN apt-get install -y lcov

# Set up CI ccache
RUN apt-get install -y ccache && ccache -M 100M 
ENV PATH="/usr/lib/ccache:${PATH}"
