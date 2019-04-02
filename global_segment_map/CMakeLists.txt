cmake_minimum_required(VERSION 2.8.3)
project(global_segment_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_definitions(-fext-numeric-literals)

cs_add_library(${PROJECT_NAME}
  src/label_block_serialization.cc
  src/label_merge_integrator.cc
)

cs_install()
cs_export()
