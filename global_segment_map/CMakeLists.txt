cmake_minimum_required(VERSION 2.8.3)
project(global_segment_map)

add_definitions(-std=c++14)
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fext-numeric-literals)
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

cs_add_library(${PROJECT_NAME}
  src/label_block_serialization.cc
  src/label_merge_integrator.cc
)

cs_install()
cs_export()
