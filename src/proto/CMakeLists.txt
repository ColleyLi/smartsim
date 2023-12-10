cmake_minimum_required(VERSION 3.20.5)
project(proto)

set(PROJECT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             sensor_msgs
)

catkin_package(INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
               DEPENDS ${GFLAGS_LIBRARIES}
)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

include(proto_gen.cmake)

include_directories(${PROJECT_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})