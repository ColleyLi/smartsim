include(${CMAKE_CURRENT_SOURCE_DIR}/proto_gen.cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../)
add_library(cm_cyber_proto ${PROTOSRCS})
target_link_libraries(cm_cyber_proto INTERFACE ${PROTOBUF_LIBRARIES})
install(TARGETS cm_cyber_proto LIBRARY DESTINATION lib)
