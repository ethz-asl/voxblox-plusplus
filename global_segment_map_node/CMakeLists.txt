cmake_minimum_required(VERSION 2.8.3)
project(gsm_node)

add_definitions(-std=c++14)
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fext-numeric-literals)
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    message(WARNING, "ApproxMVBB is not available for this cmake version and will be disabled.")
else()
    find_package(approxmvbb_catkin)
    
    if (NOT ${approxmvbb_catkin_FOUND})
      message(WARNING, "ApproxMVBB is not found, trying to build it.")
      execute_process (
          COMMAND bash -c "catkin build approxmvbb_catkin"
      )
      find_package(approxmvbb_catkin)
    endif()

    if (${approxmvbb_catkin_FOUND})
      message("ApproxMVBB is found.")
      add_definitions(-DAPPROXMVBB_AVAILABLE)
      include_directories(include ${catkin_INCLUDE_DIRS} ${approxmvbb_catkin_INCLUDE_DIRS} )
    else()
      message(WARNING, "ApproxMVBB is not found and could not build it. Please place approxmvbb_catkin package in your current workspace to have support for the bounding boxes in GSM.")
    endif()
endif()

cs_add_library(${PROJECT_NAME}_library
  src/controller.cpp
  src/sliding_window_controller.cpp
)
target_link_libraries(${PROJECT_NAME}_library ${catkin_LIBRARIES} ${approxmvbb_catkin_LIBRARIES})

cs_add_executable(${PROJECT_NAME}
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_library)

cs_add_executable(${PROJECT_NAME}_util
  src/pose_to_tf_node.cpp
)
target_link_libraries(${PROJECT_NAME}_util ${PROJECT_NAME}_library)


cs_install()
cs_export()
