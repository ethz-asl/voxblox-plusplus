cmake_minimum_required(VERSION 2.8.3)
project(global_feature_map)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

cs_add_library(${PROJECT_NAME}
  src/feature_block.cc
  src/feature_layer.cc
  src/feature_types.cc
  src/feature_integrator.cc
)

catkin_add_gtest(test_feature_layer
  test/test_feature_map.cc
)
target_link_libraries(test_feature_layer ${PROJECT_NAME})

cs_install()
cs_export()
