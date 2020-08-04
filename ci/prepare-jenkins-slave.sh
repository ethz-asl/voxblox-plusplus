#!/bin/bash -e
echo "Running the prepare script for voxblox-plusplus.";

# Install protobuf.
sudo apt-get install protobuf-compiler libprotoc-dev

# Set C++14.
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14
