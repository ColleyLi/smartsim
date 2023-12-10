cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(smartsim)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(src)