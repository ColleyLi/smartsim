find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDEDIR})

file(GLOB_RECURSE BRIDGE_COMMON_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cc)
add_library(bridge_dependencies SHARED
            ${BRIDGE_COMMON_SRCS})
target_link_libraries(bridge_dependencies
        ${PROTOBUF_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        smartsim
)

install(TARGETS bridge_dependencies
        LIBRARY DESTINATION lib
)