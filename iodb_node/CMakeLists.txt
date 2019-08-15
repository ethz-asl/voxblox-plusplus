cmake_minimum_required(VERSION 2.8.3)
project(iodb_node)

add_definitions(-std=c++14)
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fext-numeric-literals)
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_library
  src/iodb_controller.cpp
  src/sliding_window_controller.cpp
  src/feature_ros_tools.cpp
)
target_link_libraries(${PROJECT_NAME}_library ${catkin_LIBRARIES})

cs_add_executable(${PROJECT_NAME}
  src/iodb_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_library)

cs_add_executable(${PROJECT_NAME}_util
  src/pose_to_tf_node.cpp
)
target_link_libraries(${PROJECT_NAME}_util ${PROJECT_NAME}_library)

cs_install()
cs_export()
