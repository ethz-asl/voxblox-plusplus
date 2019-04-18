#!/bin/bash -e
echo "Running the prepare script for voxblox_gsm.";
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 --merge-devel

sudo apt-get install bc
CMAKE_VERSION=$(cmake --version | sed -ne 's/[^0-9]*\(\([0-9]\.\)\{0,4\}[0-9][^.]\).*/\1/p')
MIN_CMAKE_VERSION=3.10
if (($(echo "${CMAKE_VERSION} >= ${MIN_CMAKE_VERSION}" | bc -l) )); then
  catkin build approxmvbb_catkin
fi
