cmake_minimum_required(VERSION 2.8.3)
project(global_segment_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL 1.8 REQUIRED)

add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}
  src/label_block_serialization.cc
)

cs_install()
cs_export()
