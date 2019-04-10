cmake_minimum_required(VERSION 2.8.3)
project(global_feature_map)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

cs_add_library(${PROJECT_NAME}
)

cs_install()
cs_export()
