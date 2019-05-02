cmake_minimum_required(VERSION 2.8.3)
project(global_feature_map)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


set(PROTO_DEFNS proto/global_feature_map/FeatureBlock.proto
                proto/global_feature_map/FeatureLayer.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()

cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})


cs_add_library(${PROJECT_NAME}
  src/feature_block.cc
  src/feature_types.cc
  src/feature_integrator.cc
)

catkin_add_gtest(test_feature_layer
  test/test_feature_map.cc
)
target_link_libraries(test_feature_layer ${PROJECT_NAME})

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS extras.cmake
          LIBRARIES ${ADDITIONAL_LIBRARIES})
