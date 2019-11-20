#!/bin/bash

if [ -z "$1" ]
  then
    echo "Must supply or tag to test"
    exit 1
fi

docker pull rikorose/gcc-cmake:gcc-8
docker run -i -t 170b4e52c2fa /bin/bash -c "cd /home;\
git clone https://gitlab.com/globalai/repo.git;\
git checkout ${1};\
cd repo;\
mkdir build;\
cd build;\
cmake .. -DCMAKE_BUILD_TYPE=Release;\
make -j12 -l12;\
cd ..;\
script/run_tests.sh"

