#!/bin/bash -e
echo "Running the prepare script for voxblox_gsm.";
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14
