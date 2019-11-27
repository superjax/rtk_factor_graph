#!/bin/bash

if [ -z "$1" ]
  then
    echo "Must supply or tag to test"
    exit 1
fi

#docker pull ci:latest
docker run -i -t ci:latest /bin/bash -c "cd /home;\
sudo apt update;\
sudo apt install -y clang-tools;\
git clone https://gitlab.com/globalai/repo.git;\
cd repo;\
git checkout ${1};\
mkdir build;\
cd build;\
scan-build cmake .. -DCMAKE_BUILD_TYPE=Release;\
scan-build make -j4 -l4;\
cd ..;\
script/run_tests.sh"

