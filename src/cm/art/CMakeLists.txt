# find_package(Protobuf REQUIRED)
# include_directories(${Protobuf_INCLUDE_DIRS})

# include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../../proto/sensor)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../../submodule/caic_interface/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cyber)
add_subdirectory(cyber)

#add_subdirectory(test)
#add_subdirectory(test_proto)
#
#add_subdirectory(tools)
