cmake_minimum_required(VERSION 2.8.3)
project(gsm_node)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_library
  src/controller.cpp
)

cs_add_executable(${PROJECT_NAME}
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

cs_install()
cs_export()
