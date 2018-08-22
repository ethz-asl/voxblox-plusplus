cmake_minimum_required(VERSION 2.8.3)
project(gsm_node)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_library
  src/controller.cpp
  src/sliding_window/sliding_window_base.cpp
  src/sliding_window/spatial_sliding_window.cpp
  src/sliding_window/time_based_sliding_window.cpp
  src/sliding_window/view_based_sliding_window.cpp
)
cs_add_executable(${PROJECT_NAME}
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

cs_add_executable(${PROJECT_NAME}_util
  src/pose_to_tf_node.cpp
)

cs_install()
cs_export()
