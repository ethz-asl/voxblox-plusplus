cmake_minimum_required(VERSION 2.8.3)
project(gsm_node)
set(CMAKE_CXX_STANDARD 14)


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_library
  src/controller.cpp
  src/sliding_window_controller.cpp
)
cs_add_executable(${PROJECT_NAME}
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

cs_add_library(${PROJECT_NAME}_utils_library
  src/controller.cpp
  src/sliding_window_controller.cpp
)
cs_add_executable(${PROJECT_NAME}_util
  src/pose_to_tf_node.cpp
)
target_link_libraries(${PROJECT_NAME}_util
  ${PROJECT_NAME}_utils_library
)

cs_install()
cs_export()
