if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/include
    )
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/include
    )
endif()

add_library(cm_art_parameter SHARED
    ./parameter_client.cc
    ./parameter_server.cc
    ./parameter.cc
)

install(TARGETS cm_art_parameter
        LIBRARY DESTINATION lib 
)