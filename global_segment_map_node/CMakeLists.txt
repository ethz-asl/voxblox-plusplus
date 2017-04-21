cmake_minimum_required(VERSION 2.8.3)
project(gsm_node)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-sign-compare -Wno-unused-value")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_library
  src/controller.cpp
)

############
# BINARIES #
############

cs_add_executable(${PROJECT_NAME}
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_library
)

##########
# EXPORT #
##########
cs_install()
cs_export()